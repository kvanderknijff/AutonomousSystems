import logging
import math
import time
from typing import Callable

from config import PLANNER_MODE, PLANNER_ROBOT_TIMEOUT
from mqtt_protocol import DEFAULT_GOAL_TOLERANCE, GOAL_ACTION_CLEAR, GOAL_ACTION_SET

logger = logging.getLogger(__name__)


def _normalize_angle(degrees: float) -> float:
    return (degrees + 180) % 360 - 180


def _bearing_degrees(dx: float, dy: float) -> float:
    return math.degrees(math.atan2(dy, dx))


def _steer_command(direct_error: float) -> str:
    """Drive toward the target using turns; avoid in-place spins."""
    abs_error = abs(direct_error)
    if abs_error < 12.0:
        return "Forward"
    if direct_error > 0:
        return "LeftTurn"
    return "RightTurn"

RobotCommandCallback = Callable[[str, str, str], None]
GoalCallback = Callable[[str, dict], None]
GoalToleranceCallback = Callable[[str], float]
FormationStrategy = Callable[
    [dict[str, tuple[float, float]], float, float],
    dict[str, tuple[float, float]],
]


class Robot:
    def __init__(self, robot_id: str, x: float, y: float, direction: float) -> None:
        self.robot_id: str = robot_id
        self.x: float = x
        self.y: float = y
        self.direction: float = direction
        self.last_update_time: float = time.time()
        self.target_x: float | None = None
        self.target_y: float | None = None
        self.goal_tolerance: float = DEFAULT_GOAL_TOLERANCE
        self.goal_seq: int = 0
        self.published_goal_seq: int = -1

    def update_position(self, x: float, y: float, direction: float) -> None:
        self.x = x
        self.y = y
        self.direction = direction
        self.last_update_time = time.time()

    def set_target(
        self,
        target_x: float,
        target_y: float,
        tolerance: float = DEFAULT_GOAL_TOLERANCE,
    ) -> None:
        self.target_x = target_x
        self.target_y = target_y
        self.goal_tolerance = tolerance
        self.goal_seq += 1

    def clear_target(self) -> None:
        self.target_x = None
        self.target_y = None
        self.published_goal_seq = -1

    def goal_payload(self) -> dict:
        assert self.target_x is not None
        assert self.target_y is not None
        return {
            "action": GOAL_ACTION_SET,
            "target_x": self.target_x,
            "target_y": self.target_y,
            "tolerance": self.goal_tolerance,
            "seq": self.goal_seq,
        }

    def clear_goal_payload(self) -> dict:
        return {"action": GOAL_ACTION_CLEAR, "seq": self.goal_seq}

    @property
    def has_target(self) -> bool:
        return self.target_x is not None and self.target_y is not None


class RobotManager:
    def __init__(
        self,
        on_command_calculated: RobotCommandCallback | None = None,
        on_goal_assigned: GoalCallback | None = None,
        timeout_threshold: float = PLANNER_ROBOT_TIMEOUT,
        planner_mode: str = PLANNER_MODE,
        goal_tolerance_for: GoalToleranceCallback | None = None,
    ) -> None:
        self.active_robots: dict[str, Robot] = {}
        self.timeout_threshold = timeout_threshold
        self.on_command_calculated: RobotCommandCallback | None = on_command_calculated
        self.on_goal_assigned: GoalCallback | None = on_goal_assigned
        self.planner_mode = planner_mode
        self._goal_tolerance_for = goal_tolerance_for

    def _tolerance_for(self, robot_id: str) -> float:
        if self._goal_tolerance_for is not None:
            return self._goal_tolerance_for(robot_id)
        return DEFAULT_GOAL_TOLERANCE

    def _filter_live_robots(self) -> dict[str, Robot]:
        current_time = time.time()
        live_pool: dict[str, Robot] = {}

        for robot_id, robot in self.active_robots.items():
            age = current_time - robot.last_update_time
            if age <= self.timeout_threshold:
                live_pool[robot_id] = robot
            else:
                logger.warning(
                    "[MANAGER] Robot %s skipped by planner: telemetry age %.1fs exceeds %.1fs",
                    robot_id,
                    age,
                    self.timeout_threshold,
                )
                robot.clear_target()

        return live_pool

    def process_incoming_message(
        self, robot_id: str, x: float, y: float, direction: float
    ) -> None:
        if robot_id in self.active_robots:
            self.active_robots[robot_id].update_position(x, y, direction)
            logger.debug(
                "[MANAGER] Updated: %s @ (%.1f, %.1f) Dir: %.1f°",
                robot_id,
                x,
                y,
                direction,
            )
        else:
            self.active_robots[robot_id] = Robot(robot_id, x, y, direction)
            logger.info("[MANAGER] Registered: %s @ (%.1f, %.1f)", robot_id, x, y)

    def publish_active_goals(self, *, force: bool = False) -> None:
        if self.planner_mode != "goals":
            return
        for robot in self.active_robots.values():
            if robot.has_target:
                self._publish_goal_if_needed(robot, force=force)

    def apply_formation(
        self,
        formation_strategy: FormationStrategy,
        center_x: float,
        center_y: float,
        *,
        publish_goals: bool = True,
    ) -> None:
        live_robots = self._filter_live_robots()
        if not live_robots:
            logger.warning("[FORMATION] Request denied: no live robots with telemetry.")
            return

        current_positions = {
            robot_id: (robot.x, robot.y) for robot_id, robot in live_robots.items()
        }
        calculated_targets = formation_strategy(current_positions, center_x, center_y)

        logger.info(
            "--- Applying formation around (%.1f, %.1f) for %d robots ---",
            center_x,
            center_y,
            len(live_robots),
        )
        for robot_id, (target_x, target_y) in calculated_targets.items():
            if robot_id in live_robots:
                tolerance = self._tolerance_for(robot_id)
                live_robots[robot_id].set_target(target_x, target_y, tolerance=tolerance)
                logger.info(
                    "[FORMATION] %s -> target (%.1f, %.1f) tol=%.1f",
                    robot_id,
                    target_x,
                    target_y,
                    tolerance,
                )
                if publish_goals and self.planner_mode == "goals":
                    self._publish_goal_if_needed(live_robots[robot_id])

    def handle_robot_report(self, robot_id: str, status: str, seq: int = 0) -> None:
        robot = self.active_robots.get(robot_id)
        if robot is None:
            return
        if status != "arrived":
            return
        if robot.has_target and seq and seq != robot.goal_seq:
            logger.debug(
                "[PLANNER] Ignoring stale arrived for %s (seq %d != %d)",
                robot_id,
                seq,
                robot.goal_seq,
            )
            return
        if robot.has_target:
            logger.info(
                "[PLANNER] %s reported arrived at (%.1f, %.1f)",
                robot_id,
                robot.target_x,
                robot.target_y,
            )
            robot.clear_target()

    def execute_path_planning(self) -> None:
        if self.planner_mode == "goals":
            self._execute_goal_mode()
            return
        self._execute_command_mode()

    def _publish_goal_if_needed(self, robot: Robot, *, force: bool = False) -> None:
        if not robot.has_target or self.on_goal_assigned is None:
            return
        if not force and robot.published_goal_seq == robot.goal_seq:
            return
        self.on_goal_assigned(robot.robot_id, robot.goal_payload())
        robot.published_goal_seq = robot.goal_seq

    def publish_clear_goal(self, robot_id: str) -> None:
        robot = self.active_robots.get(robot_id)
        if robot is None or self.on_goal_assigned is None:
            return
        self.on_goal_assigned(robot_id, robot.clear_goal_payload())
        robot.clear_target()

    def _execute_goal_mode(self) -> None:
        live_robots = self._filter_live_robots()
        if not live_robots:
            return

        for robot in live_robots.values():
            if robot.has_target:
                self._publish_goal_if_needed(robot)

    def _execute_command_mode(self) -> None:
        live_robots = self._filter_live_robots()
        if not live_robots:
            return

        logger.debug(
            "--- Start Path Planning Cycle (%d robots active) ---",
            len(live_robots),
        )

        for robot_id, robot in live_robots.items():
            if not robot.has_target:
                continue

            assert robot.target_x is not None
            assert robot.target_y is not None

            dx = robot.target_x - robot.x
            dy = robot.target_y - robot.y
            distance = math.hypot(dx, dy)

            direct_bearing = _bearing_degrees(dx, dy)
            direct_error = _normalize_angle(direct_bearing - robot.direction)

            if distance < robot.goal_tolerance:
                logger.info(
                    "[PLANNER] %s arrived at (%.1f, %.1f)",
                    robot_id,
                    robot.target_x,
                    robot.target_y,
                )
                robot.clear_target()
                if self.on_command_calculated:
                    self.on_command_calculated(robot_id, "STOP", "commands")
                continue

            calculated_command = _steer_command(direct_error)

            logger.info(
                "[PLANNER] %s | Dist: %.1f | Pos: (%.1f, %.1f) | Goal: (%.1f, %.1f) | "
                "Dir: %.1f° | Bearing: %.1f° | Error: %.1f° -> %s",
                robot_id,
                distance,
                robot.x,
                robot.y,
                robot.target_x,
                robot.target_y,
                robot.direction,
                direct_bearing,
                direct_error,
                calculated_command,
            )

            if self.on_command_calculated:
                self.on_command_calculated(robot_id, calculated_command, "commands")
