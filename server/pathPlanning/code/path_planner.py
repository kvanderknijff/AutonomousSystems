import logging
import math
from typing import Callable
from avoidance import calculate_apf_heading  # Import APF algorithm

# Set to logging.DEBUG for development, logging.WARNING for production
logging.basicConfig(level=logging.DEBUG, format="%(message)s")

# Type alias: maps active robot positions to their optimal formation targets
FormationStrategy = Callable[[dict[str, tuple[float, float]], float, float], dict[str, tuple[float, float]]]

class Robot:
    def __init__(self, robot_id: str, x: float, y: float, direction: float) -> None:
        self.robot_id: str = robot_id
        self.x: float = x
        self.y: float = y
        self.direction: float = direction

        # Goal coordinates assigned by the formation strategy
        self.target_x: float | None = None
        self.target_y: float | None = None

    def update_position(self, x: float, y: float, direction: float) -> None:
        """Update the current state vectors of the robot."""
        self.x = x
        self.y = y
        self.direction = direction

    def set_target(self, target_x: float, target_y: float) -> None:
        """Assign a new goal coordinate to navigate towards."""
        self.target_x = target_x
        self.target_y = target_y

    def clear_target(self) -> None:
        """Reset targets when destination is reached or mission is aborted."""
        self.target_x = None
        self.target_y = None

    @property
    def has_target(self) -> bool:
        """Check if the robot currently has an active target assigned."""
        return self.target_x is not None and self.target_y is not None


class RobotManager:
    def __init__(self, on_command_calculated: Callable[[str, str], None] | None = None) -> None:
        # Central memory database for tracking all discovered fleet states
        self.active_robots: dict[str, Robot] = {}

        # Network pipeline callback function (e.g., linked to MQTT publish)
        self.on_command_calculated: Callable[[str, str], None] | None = on_command_calculated


    def process_incoming_message(self, robot_id: str, x: float, y: float, direction: float) -> None:
        """Process incoming state updates from telemetry feed."""

        # The robot already exists, update the position
        if robot_id in self.active_robots:
            self.active_robots[robot_id].update_position(x, y, direction)
            logging.debug(f"[MANAGER] Updated: {robot_id} @ ({x:.1f}, {y:.1f})")
        
        # New robot detected, create a new object
        else:
            new_robot = Robot(robot_id, x, y, direction)
            self.active_robots[robot_id] = new_robot
            logging.debug(f"[MANAGER] Registered: {robot_id} @ ({x:.1f}, {y:.1f})")


    def apply_formation(self, formation_strategy: FormationStrategy, center_x: float, center_y: float) -> None:
        """Map active robots to structural shape coordinates using the provided strategy."""
        
        if not self.active_robots:
            logging.warning("[FORMATION] Request denied: No active robots online.")
            return
        
        # Extract current position vectors for optimization input
        current_positions: dict[str, tuple[float, float]] = {
            robot_id: (robot.x, robot.y) for robot_id, robot in self.active_robots.items()
        }

        # Compute optimal coordinate matching
        calculated_targets = formation_strategy(current_positions, center_x, center_y)

        # Assign targets to the tracking state
        logging.debug(f"\n--- Applying Formation Strategy around ({center_x:.1f}, {center_y:.1f}) ---")
        for robot_id, (target_x, target_y) in calculated_targets.items():
            if robot_id in self.active_robots:
                self.active_robots[robot_id].set_target(target_x, target_y)
                logging.debug(f"[FORMATION] Set: {robot_id} -> Target ({target_x:.1f}, {target_y:.1f})")


    def execute_path_planning(self) -> None:
        """Run loop over active fleet to compute reactive collision-free motion commands."""
        if not self.active_robots:
            return
        
        logging.debug(f"\n--- Start Path Planning Cycle ({len(self.active_robots)} robots active) ---")
        for robot_id, robot in self.active_robots.items():
            # robot.x, robot.y, and robot.direction are always the newest values here!

            # Enforce active safety stop on idle hardware
            if not robot.has_target:
                logging.debug(f"[PLANNER] {robot_id} | Idle -> STOP")
                if self.on_command_calculated:
                    self.on_command_calculated(robot_id, "STOP")
                continue

            assert robot.target_x is not None
            assert robot.target_y is not None

            # 1. Calculate distance vector to target
            dx: float = robot.target_x - robot.x
            dy: float = robot.target_y - robot.y
            distance: float = math.hypot(dx, dy)

            # 2. Arrival deadband validation
            if distance < 2.0:
                logging.debug(f"[PLANNER] {robot_id} | Arrived -> STOP")
                robot.clear_target()
                if self.on_command_calculated:
                    self.on_command_calculated(robot_id, "STOP")
                continue

            # 3. Gather coordinate positions of external neighbors for APF
            other_robots_positions: list[tuple[float, float]] = [
                (other.x, other.y) for other_id, other in self.active_robots.items() if other_id != robot_id
            ]

            # 4. Compute modified heading using Artificial Potential Fields
            # This incorporates repulsive fields from neighboring robots dynamically
            target_heading_deg: float = calculate_apf_heading(
                current_x=robot.x,
                current_y=robot.y,
                target_x=robot.target_x,
                target_y=robot.target_y,
                other_robots_positions=other_robots_positions,
                influence_radius=30.0  # Safe distance threshold
            )

            # 5. Delta calculation and normalization to [-180, 180]
            heading_error: float = target_heading_deg - robot.direction
            heading_error = (heading_error + 180) % 360 - 180  # Normalize to [-180, 180]

            # 6. Advanced Threshold Controller (Rotate vs Turn vs Forward)
            curve_threshold: float = 10.0   # Small errors -> Drive straight with micro-adjustments
            rotate_threshold: float = 45.0  # Large errors -> Pivot on the spot first

            # Dynamic controller scaling: override rotations when closing in on targets
            if distance < 10.0:
                if heading_error > curve_threshold:         calculated_command = "LeftTurn"
                elif heading_error < -curve_threshold:      calculated_command = "RightTurn"
                else:                                       calculated_command = "Forward"
            else:
                if heading_error > rotate_threshold:        calculated_command = "LeftRotate"
                elif heading_error < -rotate_threshold:     calculated_command = "RightRotate"
                elif heading_error > curve_threshold:       calculated_command = "LeftTurn"
                elif heading_error < -curve_threshold:      calculated_command = "RightTurn"
                else:                                       calculated_command = "Forward"

            logging.debug(f"[PLANNER] {robot_id} | Dist: {distance:.1f} | Error: {heading_error:.1f}° -> {calculated_command}")
            
            # Send the command back through the callback if it is registered
            # Fire data pipeline callback trigger
            if self.on_command_calculated:
                self.on_command_calculated(robot_id, calculated_command)