# Central dictionary to store all robots:
#  
# A central database in Python's memory
# The Key is a string (ID), the Value is the Robot object
# active_robots: dict[str, Robot] = {}
#
# def process_incoming_message(robot_id: str, x: float, y: float, direction: float) -> None:
#     global active_robots
#
#     if robot_id in active_robots:
#         # The robot already exists! We only update the X, Y, and direction
#         active_robots[robot_id].update_position(x, y, direction)
#         print(f"Robot {robot_id} updated to X:{x}, Y:{y}")
#     else:
#         # This is a new robot (e.g., number 5 that just turned on). 
#         # We create a new object and store it.
#         new_robot_object = Robot(robot_id, x, y, direction)
#         active_robots[robot_id] = new_robot_object
#         print(f"New robot {robot_id} registered!")
# --------------------------------------------------------------------------------

import math
from typing import Callable

# Type alias for mypy: expects robot IDs, center X/Y, and returns mapped target coordinates
FormationStrategy = Callable[[list[str], float, float], dict[str, tuple[float, float]]]


class Robot:
    def __init__(self, robot_id: str, x: float, y: float, direction: float) -> None:
        self.robot_id: str = robot_id
        self.x: float = x
        self.y: float = y
        self.direction: float = direction

        # Target coordinates (Optional[float] allows None when idle)
        self.target_x: float | None = None
        self.target_y: float | None = None

    def update_position(self, x: float, y: float, direction: float) -> None:
        """Update the coordinates of this specific robot."""
        self.x = x
        self.y = y
        self.direction = direction

    def set_target(self, target_x: float, target_y: float) -> None:
        """Set a new target destination for the path planner to process."""
        self.target_x = target_x
        self.target_y = target_y

    def clear_target(self) -> None:
        """Clear the target when the robot has reached its destination."""
        self.target_x = None
        self.target_y = None

    @property
    def has_target(self) -> bool:
        """Helper property to quickly check if the robot needs routing."""
        return self.target_x is not None and self.target_y is not None


class RobotManager:
    # VOEG HIER de parameter toe tussen de haakjes, inclusief de type hint en de defaultwaarde (= None)
    def __init__(self, on_command_calculated: Callable[[str, str], None] | None = None) -> None:
        # This is the central database in Python's memory for all robots
        self.active_robots: dict[str, Robot] = {}

        # Sla de binnengekomen parameter op in het object
        self.on_command_calculated: Callable[[str, str], None] | None = on_command_calculated



    def process_incoming_message(self, robot_id: str, x: float, y: float, direction: float) -> None:
        """This function is called as soon as MQTT data arrives."""
        if robot_id in self.active_robots:
            # The robot already exists, update the position
            self.active_robots[robot_id].update_position(x, y, direction)
            print(f"Robot {robot_id} updated to X:{x}, Y:{y}, Direction:{direction}")
        else:
            # New robot detected, create a new object
            new_robot = Robot(robot_id, x, y, direction)
            self.active_robots[robot_id] = new_robot
            print(f"New robot {robot_id} registered at X:{x}, Y:{y}")

    def apply_formation(self, formation_strategy: FormationStrategy, center_x: float, center_y: float) -> None:
        """Gathers all online robots and maps targets using the provided strategy."""
        robot_ids = list(self.active_robots.keys())
        if not robot_ids:
            print("No active robots available for formation assignment.")
            return
        
        # Execute the injected strategy (Dependency Injection)
        calculated_targets = formation_strategy(robot_ids, center_x, center_y)

        # Assign targets to our tracking state
        for robot_id, (target_x, target_y) in calculated_targets.items():
            if robot_id in self.active_robots:
                self.active_robots[robot_id].set_target(target_x, target_y)
                print(f"Formation target set for {robot_id} -> Target X: {target_x:.1f}, Y: {target_y:.1f}")

    def execute_path_planning(self) -> None:
        """Calculate advanced navigation commands including positioning, rotations, and turning arcs."""
        if not self.active_robots:
            return
        
        print(f"\n--- Start Path Planning Cycle ({len(self.active_robots)} robots active) ---")
        for robot_id, robot in self.active_robots.items():
            # robot.x, robot.y, and robot.direction are always the newest values here!

            if not robot.has_target:
                print(f"Robot {robot_id} has no target. Standing by.")
                continue

            assert robot.target_x is not None
            assert robot.target_y is not None

            # 1. Calculate vectors and distance
            dx: float = robot.target_x - robot.x
            dy: float = robot.target_y - robot.y
            distance: float = math.hypot(dx, dy)

            # 2. Arrival validation (Deadband / Threshold of 2.0 units)
            if distance < 2.0:
                print(f"Robot {robot_id} has arrived at destination! Clearing target.")
                robot.clear_target()
                if self.on_command_calculated:
                    self.on_command_calculated(robot_id, "STOP")
                continue

            # 3. Heading calculations
            target_heading_deg: float = math.degrees(math.atan2(dy, dx))
            heading_error: float = target_heading_deg - robot.direction
            heading_error = (heading_error + 180) % 360 - 180  # Normalize to [-180, 180]

            # 4. Advanced Threshold Controller (Rotate vs Turn vs Forward)
            curve_threshold: float = 10.0   # Small errors -> Drive straight with micro-adjustments
            rotate_threshold: float = 45.0  # Large errors -> Pivot on the spot first

            if heading_error > rotate_threshold:
                calculated_command = "LeftRotate"
            elif heading_error < -rotate_threshold:
                calculated_command = "RightRotate"
            elif heading_error > curve_threshold:
                calculated_command = "LeftTurn"
            elif heading_error < -curve_threshold:
                calculated_command = "RightTurn"
            else:
                calculated_command = "Forward"

            print(f"[{robot_id}] Dist: {distance:.1f} | Error Angle: {heading_error:.1f}° -> Action: {calculated_command}")
            
            # Send the command back via the callback if it is registered
            if self.on_command_calculated:
                self.on_command_calculated(robot_id, calculated_command)