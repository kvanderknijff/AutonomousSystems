"""Event-driven bridge: plan on each camera position update, formations on web click."""

import logging
import threading
from functools import partial
from typing import TYPE_CHECKING, Any

from formations import calculate_line, calculate_plus, calculate_square
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
    - Each camera position update -> process_incoming_message + execute_path_planning
    - Web formation click -> apply_formation once (targets injected into the manager)
    """

    def __init__(
        self,
        server: "CentralServer",
        center_x: float = 320.0,
        center_y: float = 240.0,
        spacing: float = 40.0,
        min_robots_for_formation: int = 2,
        command_resend_interval: int = 4,
        formation_auto_center: bool = True,
        initial_formation: str | None = None,
    ) -> None:
        self.server = server
        self.default_center_x = center_x
        self.default_center_y = center_y
        self.default_spacing = spacing
        self.min_robots_for_formation = min_robots_for_formation
        self.formation_auto_center = formation_auto_center

        self._lock = threading.Lock()
        self._pending_formation: dict[str, Any] | None = None
        self._stop_requested = False
        self._current_formation: str | None = None
        self._last_commands: dict[str, str] = {}
        self._command_cycles: dict[str, int] = {}
        self._command_resend_interval = max(1, command_resend_interval)
        self._update_count = 0

        self._manager = RobotManager(on_command_calculated=self._on_publish)

        if initial_formation and initial_formation in FORMATION_STRATEGIES:
            self.request_formation(
                initial_formation,
                center_x=center_x,
                center_y=center_y,
                spacing=spacing,
            )

    def attach(self, server: "CentralServer") -> None:
        """Wire planner to live MQTT position stream (call before server.start())."""
        server.set_position_listener(self.on_position_update)
        logger.info("Path planner attached to camera telemetry stream")

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
                if mqtt_code != "SS" and cycles < self._command_resend_interval:
                    return
            else:
                self._command_cycles[robot_id] = 0

        if self.server.publish(robot_id, mqtt_code, "command"):
            with self._lock:
                self._last_commands[robot_id] = mqtt_code
                self._command_cycles[robot_id] = 0

    def on_position_update(self, mac: str, x: float, y: float, orientation: float) -> None:
        """Scenario A: high-frequency camera telemetry — update state and plan immediately."""
        with self._lock:
            self._update_count += 1
            self._prune_disconnected_robots()
            self._manager.process_incoming_message(mac, x, y, orientation)
            self._try_apply_pending_formation()

        # Run planning outside the lock — send_command needs the central server lock.
        self._manager.execute_path_planning()

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

        logger.info("Formation applied immediately: %s", payload)
        return payload

    def stop_all(self) -> None:
        with self._lock:
            self._pending_formation = None
            self._stop_requested = True
            self._current_formation = None
            for robot in self._manager.active_robots.values():
                robot.clear_target()
        for mac in self.server.get_connected_positions():
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

    def get_status(self) -> dict[str, Any]:
        positions = self.server.get_connected_positions()
        with self._lock:
            self._prune_disconnected_robots()
            robots = [
                {
                    "mac": mac,
                    "x": info["x"],
                    "y": info["y"],
                    "orientation": info["orientation"],
                    "has_target": self._manager.active_robots[mac].has_target
                    if mac in self._manager.active_robots
                    else False,
                }
                for mac, info in positions.items()
            ]
            planner_only = [
                mac
                for mac in self._manager.active_robots
                if mac not in positions
            ]
            tracked_robots = len(self._manager.active_robots)
        suggested_center = None
        if robots:
            suggested_center = {
                "x": round(sum(r["x"] for r in robots) / len(robots)),
                "y": round(sum(r["y"] for r in robots) / len(robots)),
            }
        return {
            "connected_robots": len(robots),
            "planner_tracked_robots": tracked_robots,
            "stale_planner_robots": planner_only,
            "current_formation": self._current_formation,
            "formations": list(FORMATION_STRATEGIES.keys()),
            "sequential": {"active": False, "current_robot": None, "step": 0, "total": 0},
            "robots": robots,
            "suggested_center": suggested_center,
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
        for stale_id in list(self._manager.active_robots.keys()):
            if stale_id not in connected_ids:
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
        self._manager.apply_formation(
            partial(strategy, spacing=payload["spacing"]),
            center_x,
            center_y,
        )
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
