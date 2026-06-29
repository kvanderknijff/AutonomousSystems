"""Event-driven bridge: plan on each camera position update, formations on web click."""

import logging
import threading
import time
from functools import partial
from typing import TYPE_CHECKING, Any

from formations import calculate_line, calculate_plus, calculate_square
from config import PLANNER_DISCONNECT_GRACE, PLANNER_MODE, FIELD_MARGIN
from path_planner import RobotManager

if TYPE_CHECKING:
    from central_server import CentralServer

logger = logging.getLogger(__name__)

COMMAND_MAP = {
    "Forward": "FW",
    "LeftTurn": "TL",
    "RightTurn": "TR",
    "LeftRotate": "RL",
    "RightRotate": "RR",
    "STOP": "SS",
}

FORMATION_STRATEGIES = {
    "line": calculate_line,
    "plus": calculate_plus,
    "square": calculate_square,
}


class PlannerController:
    """
    Mirrors the colleague's main_server pattern:
    - One persistent RobotManager for the process lifetime
    - Each camera position update -> enqueue telemetry for the planner worker
    - Web formation click -> apply_formation once (targets injected into the manager)
    """

    def __init__(
        self,
        server: "CentralServer",
        center_x: float = 320.0,
        center_y: float = 240.0,
        spacing: float = 40.0,
        min_robots_for_formation: int = 1,
        command_resend_interval: int = 4,
        formation_auto_center: bool = True,
        initial_formation: str | None = None,
        planner_interval: float = 0.25,
    ) -> None:
        self.server = server
        self.default_center_x = center_x
        self.default_center_y = center_y
        self.default_spacing = spacing
        self.min_robots_for_formation = min_robots_for_formation
        self.formation_auto_center = formation_auto_center
        self.planner_interval = max(0.0, planner_interval)

        self._lock = threading.RLock()
        self._pending_formation: dict[str, Any] | None = None
        self._pending_positions: dict[str, tuple[float, float, float]] = {}
        self._position_event = threading.Event()
        self._worker_thread: threading.Thread | None = None
        self._shutdown_requested = False
        self._stop_requested = False
        self._current_formation: str | None = None
        self._last_commands: dict[str, str] = {}
        self._command_cycles: dict[str, int] = {}
        self._command_resend_interval = max(1, command_resend_interval)
        self._disconnect_grace = PLANNER_DISCONNECT_GRACE
        self._update_count = 0
        self._last_plan_time = 0.0
        self._field_bounds_ready = False

        self._manager = RobotManager(
            on_command_calculated=self._on_publish,
            on_goal_assigned=self._on_goal_assigned,
            planner_mode=PLANNER_MODE,
        )

        if initial_formation and initial_formation in FORMATION_STRATEGIES:
            self.request_formation(
                initial_formation,
                center_x=center_x,
                center_y=center_y,
                spacing=spacing,
            )

    def attach(self, server: "CentralServer") -> None:
        """Wire planner to live MQTT position stream (call before server.start())."""
        self._start_worker()
        server.set_position_listener(self.on_position_update)
        server.set_report_listener(self.on_robot_report)
        server.set_field_bounds_listener(self.on_field_bounds_update)
        logger.info(
            "Path planner attached to camera telemetry stream (mode=%s)",
            PLANNER_MODE,
        )

    def stop(self) -> None:
        with self._lock:
            self._shutdown_requested = True
        self._position_event.set()
        if self._worker_thread is not None:
            self._worker_thread.join(timeout=2.0)

    def _on_publish(self, robot_id: str, payload: str, topic_type: str) -> None:
        """Central routing: movement commands vs connection status (Kai's 3-arg callback)."""
        if topic_type == "status":
            if self.server.publish(robot_id, payload, "status"):
                logger.debug("Published status '%s' for %s", payload, robot_id)
            return

        mqtt_code = COMMAND_MAP.get(payload)
        if mqtt_code is None:
            logger.warning("Unmapped planner command for %s: %s", robot_id, payload)
            return

        with self._lock:
            last = self._last_commands.get(robot_id)
            if last == mqtt_code:
                cycles = self._command_cycles.get(robot_id, 0) + 1
                self._command_cycles[robot_id] = cycles
                if cycles < self._command_resend_interval:
                    return
            else:
                self._command_cycles[robot_id] = 0

        if self.server.publish(robot_id, mqtt_code, "command"):
            with self._lock:
                self._last_commands[robot_id] = mqtt_code
                self._command_cycles[robot_id] = 0

    def _on_goal_assigned(self, robot_id: str, goal: dict) -> None:
        if self.server.publish_goal(robot_id, goal):
            logger.info("[GOAL] %s -> %s", robot_id, goal)

    def on_robot_report(self, mac: str, report: dict) -> None:
        with self._lock:
            self._manager.handle_robot_report(mac, report.get("status", ""), report.get("seq", 0))

    def on_position_update(self, mac: str, x: float, y: float, orientation: float) -> None:
        """Scenario A: high-frequency camera telemetry — enqueue work and return quickly."""
        with self._lock:
            self._update_count += 1
            self._pending_positions[mac] = (x, y, orientation)
        self._position_event.set()

    def on_field_bounds_update(self) -> None:
        """Re-clamp active goals when corner markers define the playfield."""
        with self._lock:
            bounds = self.server.get_field_bounds()
            became_ready = bounds.is_ready and not self._field_bounds_ready
            self._field_bounds_ready = bounds.is_ready
            changed = self._clamp_robot_targets_to_field()
        if became_ready:
            rect = bounds.bounds()
            if rect is not None:
                logger.info(
                    "Field bounds ready: (%.0f, %.0f) – (%.0f, %.0f)",
                    rect[0],
                    rect[1],
                    rect[2],
                    rect[3],
                )
        if changed or became_ready:
            logger.info("Field bounds updated — reclamped robot targets")

    def _start_worker(self) -> None:
        if self._worker_thread is not None and self._worker_thread.is_alive():
            return
        with self._lock:
            self._shutdown_requested = False
        self._worker_thread = threading.Thread(
            target=self._planner_worker,
            name="planner-worker",
            daemon=True,
        )
        self._worker_thread.start()

    def _planner_worker(self) -> None:
        """Process telemetry outside the MQTT callback thread."""
        while True:
            self._position_event.wait(timeout=0.1)
            with self._lock:
                if self._shutdown_requested:
                    return
                pending_positions = self._pending_positions
                self._pending_positions = {}
                self._position_event.clear()

            if not pending_positions:
                continue

            should_plan = self._process_position_batch(pending_positions)
            if should_plan:
                self._run_planning_cycle()

    def _run_planning_cycle(self) -> None:
        with self._lock:
            self._manager.execute_path_planning()

    def _process_position_batch(
        self,
        positions: dict[str, tuple[float, float, float]],
    ) -> bool:
        """Apply queued telemetry and decide whether this batch should trigger planning."""
        with self._lock:
            should_plan = False
            for mac, (x, y, orientation) in positions.items():
                self._manager.process_incoming_message(mac, x, y, orientation)
            self._try_apply_pending_formation()
            self._prune_disconnected_robots()
            now = time.time()
            if now - self._last_plan_time >= self.planner_interval:
                self._last_plan_time = now
                should_plan = True
            return should_plan

    def request_formation(
        self,
        formation: str,
        center_x: float | None = None,
        center_y: float | None = None,
        spacing: float | None = None,
        auto_center: bool | None = None,
    ) -> dict[str, Any]:
        """Scenario B: one-time web click — queue or apply formation immediately."""
        formation_key = formation.strip().lower()
        if formation_key not in FORMATION_STRATEGIES:
            raise ValueError(
                f"Unknown formation '{formation}'. Use: {', '.join(FORMATION_STRATEGIES)}"
            )

        payload = {
            "formation": formation_key,
            "center_x": center_x if center_x is not None else self.default_center_x,
            "center_y": center_y if center_y is not None else self.default_center_y,
            "spacing": spacing if spacing is not None else self.default_spacing,
            "auto_center": self.formation_auto_center if auto_center is None else auto_center,
        }

        with self._lock:
            self._stop_requested = False
            positions = self.server.get_connected_positions()
            if len(positions) < self.min_robots_for_formation:
                self._pending_formation = payload
                logger.info(
                    "Formation queued (waiting for %d/%d robots): %s",
                    len(positions),
                    self.min_robots_for_formation,
                    payload,
                )
                return payload

            self._sync_all_positions(positions)
            self._apply_formation_payload(payload)

        self._run_planning_cycle()
        logger.info("Formation applied immediately: %s", payload)
        return payload

    def stop_all(self) -> None:
        with self._lock:
            self._pending_formation = None
            self._stop_requested = True
            self._current_formation = None
            robot_ids = list(self._manager.active_robots.keys())
            for robot in self._manager.active_robots.values():
                robot.clear_target()
        for mac in robot_ids:
            if PLANNER_MODE == "goals":
                self._manager.publish_clear_goal(mac)
            self.server.send_command(mac, "SS")
            self._last_commands[mac] = "SS"
        for mac in self.server.get_connected_positions():
            if mac not in robot_ids:
                if PLANNER_MODE == "goals":
                    self.server.publish_goal(mac, {"action": "clear", "seq": 0})
                self.server.send_command(mac, "SS")
                self._last_commands[mac] = "SS"
        logger.info("Stop all robots requested")

    def _resolve_formation_center(
        self,
        positions: dict[str, dict],
        center_x: float,
        center_y: float,
        auto_center: bool,
    ) -> tuple[float, float]:
        if not positions:
            return center_x, center_y

        xs = [float(info["x"]) for info in positions.values()]
        ys = [float(info["y"]) for info in positions.values()]
        centroid_x = sum(xs) / len(xs)
        centroid_y = sum(ys) / len(ys)

        if auto_center:
            logger.info(
                "Using fleet centroid (%.0f, %.0f) for formation center",
                centroid_x,
                centroid_y,
            )
            return centroid_x, centroid_y

        return center_x, center_y

    def _clamp_robot_targets_to_field(self) -> bool:
        bounds = self.server.get_field_bounds()
        if not bounds.is_ready:
            return False
        changed = False
        for robot in self._manager.active_robots.values():
            if not robot.has_target:
                continue
            assert robot.target_x is not None
            assert robot.target_y is not None
            clamped_x, clamped_y = bounds.clamp_point(
                robot.target_x,
                robot.target_y,
                FIELD_MARGIN,
            )
            if clamped_x != robot.target_x or clamped_y != robot.target_y:
                logger.info(
                    "[FIELD] %s target clamped (%.1f, %.1f) -> (%.1f, %.1f)",
                    robot.robot_id,
                    robot.target_x,
                    robot.target_y,
                    clamped_x,
                    clamped_y,
                )
                robot.set_target(clamped_x, clamped_y, tolerance=robot.goal_tolerance)
                changed = True
                if PLANNER_MODE == "goals":
                    self._manager._publish_goal_if_needed(robot, force=True)
        return changed

    def _robot_display_state(self, mac: str, info: dict[str, Any]) -> dict[str, Any]:
        """Merge registry + planner telemetry for the web UI."""
        planner = self._manager.active_robots.get(mac)
        now = time.time()
        if planner is not None:
            telemetry_age = now - planner.last_update_time
            has_live_position = telemetry_age <= self._manager.timeout_threshold
            x = planner.x
            y = planner.y
            orientation = planner.direction
            has_target = planner.has_target
            target_x = planner.target_x
            target_y = planner.target_y
        else:
            telemetry_age = None
            has_live_position = bool(info.get("x") or info.get("y"))
            x = float(info.get("x", 0))
            y = float(info.get("y", 0))
            orientation = float(info.get("orientation", 0))
            has_target = False
            target_x = None
            target_y = None

        return {
            "mac": mac,
            "aruco_id": info.get("aruco_id"),
            "x": round(x, 1),
            "y": round(y, 1),
            "orientation": round(orientation, 1),
            "has_live_position": has_live_position,
            "telemetry_age_sec": round(telemetry_age, 1) if telemetry_age is not None else None,
            "has_target": has_target,
            "target_x": round(target_x, 1) if target_x is not None else None,
            "target_y": round(target_y, 1) if target_y is not None else None,
        }

    def get_status(self) -> dict[str, Any]:
        positions = self.server.get_connected_positions()
        with self._lock:
            self._prune_disconnected_robots()
            robots = [
                self._robot_display_state(mac, info)
                for mac, info in positions.items()
            ]
            live_robots = [robot for robot in robots if robot["has_live_position"]]
            planner_only = [
                mac
                for mac in self._manager.active_robots
                if mac not in positions
            ]
            tracked_robots = len(self._manager.active_robots)
        suggested_center = None
        if live_robots:
            suggested_center = {
                "x": round(sum(r["x"] for r in live_robots) / len(live_robots)),
                "y": round(sum(r["y"] for r in live_robots) / len(live_robots)),
            }
        elif robots:
            suggested_center = {
                "x": round(sum(r["x"] for r in robots) / len(robots)),
                "y": round(sum(r["y"] for r in robots) / len(robots)),
            }
        field_bounds = self.server.get_field_bounds()
        field_center = field_bounds.center() if field_bounds.is_ready else None
        if suggested_center is None and field_center is not None:
            suggested_center = {
                "x": round(field_center[0]),
                "y": round(field_center[1]),
            }
        return {
            "connected_robots": len(robots),
            "planner_tracked_robots": tracked_robots,
            "stale_planner_robots": planner_only,
            "current_formation": self._current_formation,
            "planner_mode": PLANNER_MODE,
            "formations": list(FORMATION_STRATEGIES.keys()),
            "sequential": {"active": False, "current_robot": None, "step": 0, "total": 0},
            "robots": robots,
            "suggested_center": suggested_center,
            "field_bounds": field_bounds.as_dict(),
            "coordinate_space": "camera_pixels",
        }

    def _sync_all_positions(self, positions: dict[str, dict]) -> None:
        for mac, info in positions.items():
            self._manager.process_incoming_message(
                mac,
                float(info["x"]),
                float(info["y"]),
                float(info["orientation"]),
            )

    def _prune_disconnected_robots(self) -> None:
        connected_ids = set(self.server.get_connected_positions().keys())
        now = time.time()
        for stale_id in list(self._manager.active_robots.keys()):
            if stale_id in connected_ids:
                continue
            robot = self._manager.active_robots[stale_id]
            telemetry_age = now - robot.last_update_time
            if telemetry_age <= self._disconnect_grace:
                continue
            del self._manager.active_robots[stale_id]
            self._last_commands.pop(stale_id, None)
            logger.info("Removed disconnected robot from planner: %s", stale_id)

    def _apply_formation_payload(self, payload: dict[str, Any]) -> None:
        positions = self.server.get_connected_positions()
        strategy = FORMATION_STRATEGIES[payload["formation"]]
        auto_center = payload.get("auto_center", self.formation_auto_center)
        center_x, center_y = self._resolve_formation_center(
            positions,
            payload["center_x"],
            payload["center_y"],
            auto_center,
        )
        bounds = self.server.get_field_bounds()
        if bounds.is_ready:
            center_x, center_y = bounds.clamp_point(center_x, center_y, FIELD_MARGIN)
            self._field_bounds_ready = True
        self._manager.apply_formation(
            partial(strategy, spacing=payload["spacing"]),
            center_x,
            center_y,
            publish_goals=False,
        )
        self._clamp_robot_targets_to_field()
        if PLANNER_MODE == "goals":
            self._manager.publish_active_goals(force=True)
        self._current_formation = payload["formation"]
        self._last_commands.clear()
        self._command_cycles.clear()
        logger.info(
            "Applied %s formation at (%.0f, %.0f) for %d robots",
            payload["formation"],
            center_x,
            center_y,
            len(positions),
        )

    def _try_apply_pending_formation(self) -> None:
        if self._stop_requested:
            self._stop_requested = False
            return
        if self._pending_formation is None:
            return

        positions = self.server.get_connected_positions()
        if len(positions) < self.min_robots_for_formation:
            return

        pending = self._pending_formation
        self._pending_formation = None
        self._sync_all_positions(positions)
        self._apply_formation_payload(pending)
        self._run_planning_cycle()
