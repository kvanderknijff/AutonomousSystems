# test_path_planner.py
import time

# Clean, native local imports since all files share the same directory
from test_logger import setup_file_logger
import formations
from path_planner import RobotManager

# Initialize the file logger to capture all debug streams into /dataOutput/
setup_file_logger()

def mock_mqtt_publisher(robot_id: str, payload: str, topic_type: str) -> None:
    """Mock callback mimicking network transmissions via the MQTT broker."""
    # We accept 3 arguments now: id, the command/status, and where it needs to go
    pass


# 1. Instantiate core coordinator
manager = RobotManager(on_command_calculated=mock_mqtt_publisher)

# 2. Define the strategic Look-Up Table for the web interface integration
formation_map = {
    "line": formations.calculate_line,
    "plus": formations.calculate_plus,
    "square": formations.calculate_square,
    "y": formations.calculate_Y,
}

def trigger_web_formation(shape_key: str, cx: float, cy: float) -> None:
    """Simulate routing incoming frontend configurations to the core engine."""
    if shape_key in formation_map:
        selected_strategy = formation_map[shape_key]
        manager.apply_formation(selected_strategy, center_x=cx, center_y=cy)
    else:
        print(f"[WEB ERROR] Requested formation string '{shape_key}' is invalid.")

# ==============================================================================
# REALISTIC EXECUTION SIMULATION LOOP
# ==============================================================================
def simulate_until_formation_complete(frame_delay: float = 0.01) -> None:
    """Simulate camera feed dynamically until all active robots reach their targets."""
    frame_count = 0
    
    while True:
        robots_moving = False
        
        for robot_id, robot in manager.active_robots.items():
            if robot.has_target:
                robots_moving = True
                assert robot.target_x is not None
                assert robot.target_y is not None
                
                step_x = 1.0 if robot.x < robot.target_x else -1.0
                step_y = 1.0 if robot.y < robot.target_y else -1.0
                
                if abs(robot.target_x - robot.x) < 1.0: step_x = robot.target_x - robot.x
                if abs(robot.target_y - robot.y) < 1.0: step_y = robot.target_y - robot.y
                
                manager.process_incoming_message(
                    robot_id=robot_id,
                    x=robot.x + step_x,
                    y=robot.y + step_y,
                    direction=(robot.direction + 2.0) % 360
                )
                
        if not robots_moving:
            print(f"--> Formation successfully completed in {frame_count} frames!")
            break
            
        manager.execute_path_planning()
        frame_count += 1
        time.sleep(frame_delay)


# --- RUN DYNAMIC TESTS ---
print("--- Step 1: Initial Fleet Registration ---")
manager.process_incoming_message("robot_1", 10.0, 10.0, 0.0)
manager.process_incoming_message("robot_2", 20.0, 50.0, 90.0)
manager.process_incoming_message("robot_3", 80.0, 10.0, 180.0)
manager.process_incoming_message("robot_4", 90.0, 90.0, 270.0)

print("\n--- Step 2: Web App Requests PLUS Formation ---")
trigger_web_formation("plus", cx=50.0, cy=50.0)
simulate_until_formation_complete()

print("Sleep for 10s");
time.sleep(10);

print("\n--- Step 3: Robot 5 Joins, Web App Requests LINE Formation ---")
manager.process_incoming_message("robot_5", 0.0, 0.0, 0.0)
trigger_web_formation("line", cx=100.0, cy=100.0)
simulate_until_formation_complete()

print("\n--- Step 4: Web App Requests New SQUARE Formation ---")
trigger_web_formation("square", cx=120.0, cy=120.0)
simulate_until_formation_complete()
