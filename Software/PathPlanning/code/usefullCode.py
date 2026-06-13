# VOORBEELD VOOR DE INTEGRATIE MET DE FRONTEND (Engelse code)
import formations

# A look-up table (map) to route web strings to Python function objects
formation_map = {
    "line": formations.calculate_line,
    "plus": formations.calculate_plus,
    # "square": formations.calculate_square
}

def handle_web_request(incoming_shape_string: str, x: float, y: float) -> None:
    """Dynamically executes the correct formation strategy based on the frontend string."""
    if incoming_shape_string in formation_map:
        # 1. Retrieve the correct function object from the map
        selected_strategy = formation_map[incoming_shape_string]
        
        # 2. Inject it straight into your manager
        manager.apply_formation(selected_strategy, center_x=x, center_y=y)
    else:
        print(f"Unknown formation requested: {incoming_shape_string}")


#----------------------------------------------------------------------------------------------------



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


#2 Adding status of robot,
# there might be 5 robots, but if only 4 are connected, only 4 need to be used. 

#3 Algorithm Hongarian
# robot location allocation

#4 APF/VO Algorithm
# best path for each robot without really crossing path

#5 turning car into right direction to accentuate the shape of the made shape. 
# and why the < 2.0 distance?
# and, need to send STOP when a robot arives at destination