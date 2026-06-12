from path_planner import RobotManager
import formations

def mock_mqtt_publisher(robot_id: str, command: str) -> None:
    print(f"[MOCK MQTT] Transmitting '{command}' to topic 'robots/{robot_id}/commands'")

# Instantiate manager
manager = RobotManager(on_command_calculated=mock_mqtt_publisher)

# 1. Simulate 4 robots connecting via camera feedback
print("--- Registering 4 Robots ---")
manager.process_incoming_message("robot_1", 10.0, 10.0, 0.0)
manager.process_incoming_message("robot_2", 20.0, 50.0, 90.0)
manager.process_incoming_message("robot_3", 80.0, 10.0, 180.0)
manager.process_incoming_message("robot_4", 90.0, 90.0, 270.0)

# 2. Trigger a PLUS formation requested by the user
print("\n--- Applying PLUS Formation at Center (50.0, 50.0) ---")
manager.apply_formation(formations.calculate_plus, center_x=50.0, center_y=50.0)

# 3. Process commands
manager.execute_path_planning()

# 4. A 5th robot connects, web app changes formation to LINE
print("\n--- Robot 5 Appears, Switching to LINE Formation at Center (100.0, 100.0) ---")
manager.process_incoming_message("robot_5", 0.0, 0.0, 0.0)
manager.apply_formation(formations.calculate_line, center_x=100.0, center_y=100.0)

# 5. Process new layout commands
manager.execute_path_planning()




## Extra info:

#1 Why this fits your web architecture:
# When you build the web interface later, 
# your backend API will receive a payload like {"formation": "plus", "x": 150, "y": 150}. 
# You can create a simple dictionary routing map to translate that string to the function call:

#        pythonformation_map = {
#            "line": formations.calculate_line,
#            "plus": formations.calculate_plus
#        }
#        # Execute dynamically
#        selected_strategy = formation_map[incoming_web_string]
#        manager.apply_formation(selected_strategy, web_x, web_y)