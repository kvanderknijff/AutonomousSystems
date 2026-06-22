import random
import paho.mqtt.client as mqtt
from controller import Robot
import threading
import json
import math
import time
import numpy as np
'''
todo controller:
- rewrite the move to position to have variable turn rate
- rewrite the pathing light:
	- ?send lightsource even at distance (not sure if that works in webots)
	- change the found to wait on a target position from the server instead of locking it down
distance:
width sim: ~24.35 real: ~2.07
height sim": ~15.85 real: ~1.17

'''
robot = Robot()

timestep = int(robot.getBasicTimeStep())
# sensors 
range_finder = robot.getDevice('range-finder')
ldr = robot.getDevice('light sensor')

#setup
last_spin_time = 0
ldr_min_threshold = 280   
ldr_max_threshold = 900

distance_scaling_factor = 0.073416 # 0.07341645385809423664313254218617
ldr.enable(timestep)
range_finder.enable(timestep)
#readings
ldr_readings = []
distance_readings = []

radius = 60
reduced_radius = 40
radius_correction = 60
robot_data = {}



# Initialize motors and sensors
motor1 = robot.getDevice('motor1')
motor2 = robot.getDevice('motor2')
motor3 = robot.getDevice('motor3')
# Set initial motor velocities
motor1.setVelocity(0)
motor2.setVelocity(0)
motor3.setVelocity(0)

motor1.setPosition(float('inf'))
motor2.setPosition(float('inf'))
motor3.setPosition(float('inf'))

# Motor velocities
forward_velocity = -3.0
backward_velocity = 3.0


# MQTT settings
broker = 'localhost'
port = 1883
client_id = f'robot_{random.randint(0, 10000)}'
topic_register = "swarm/register"
client = None
topics = {}

notfinished = True
target_position = None

target_tolerance = 20
reduced_avoidance_radius = 300

def euclidian_distance(vector1, vector2):
    return np.linalg.norm(np.array(vector1) - np.array(vector2))


# Movement functions
def MoveForward(duration):
    print("MoveForward called with duration:", duration)
    motor1.setVelocity(forward_velocity)
    motor2.setVelocity(forward_velocity)
    end_time = robot.getTime() + duration
    while robot.getTime() < end_time:
        if robot.step(timestep) == -1:
            break
    motor1.setVelocity(0)
    motor2.setVelocity(0)

def MoveForwardCont():
    print("MoveForwardCont called")
    motor1.setVelocity(forward_velocity)
    motor2.setVelocity(forward_velocity)
# Movement functions
def Stop():
    print("Stop")
    motor1.setVelocity(0)
    motor2.setVelocity(0)

def MoveBack(duration):
    print("MoveBack called with duration:", duration)
    motor1.setVelocity(backward_velocity)
    motor2.setVelocity(backward_velocity)
    end_time = robot.getTime() + duration
    while robot.getTime() < end_time:
        if robot.step(timestep) == -1:
            break
    motor1.setVelocity(0)
    motor2.setVelocity(0)

def SpinLeft(duration):
    print("SpinLeft called with duration:", duration)
    motor2.setVelocity(backward_velocity)
    motor1.setVelocity(forward_velocity)
    end_time = robot.getTime() + duration
    while robot.getTime() < end_time:
        if robot.step(timestep) == -1:
            break
    motor1.setVelocity(0)
    motor2.setVelocity(0)

def SpinRight(duration):
    print("SpinRight called with duration:", duration)
    motor1.setVelocity(backward_velocity)
    motor2.setVelocity(forward_velocity)
    end_time = robot.getTime() + duration
    while robot.getTime() < end_time:
        if robot.step(timestep) == -1:
            break
    motor1.setVelocity(0)
    motor2.setVelocity(0)

def SpinTop(speed, duration):
    global ldr_readings
    global distance_readings
    step_count = 10
    angle_step = 180 / (step_count - 1)
    initial_position = -90
    ldr_readings = []
    distance_readings = []
    angles = []

    for step in range(step_count):
        position_degrees = initial_position + step * angle_step
        position_radians = position_degrees * (math.pi / 180)
        motor3.setPosition(position_radians)
        motor3.setVelocity(speed)
        end_time = robot.getTime() + duration
        while robot.getTime() < end_time:
            if robot.step(timestep) == -1:
                break
        ldr_value = ldr.getValue()
        ldr_readings.append(ldr_value)
        distance_image = range_finder.getRangeImage()
        distance_value = process_range_image(distance_image)
        distance_readings.append(distance_value)
        angles.append(position_degrees)
        
    motor3.setPosition(0)
    return ldr_readings, distance_readings, angles
    

def process_range_image(image):
    if len(image) == 0:
        return float('inf')
    min_distance = float('inf')
    for distance in image:
        if distance < min_distance:
            min_distance = distance
    return min_distance
    

def calculate_intersection_points(coord1, coord2, radius):
    """Calculate the intersection points of two bots given their positions and direction vectors."""
    coord1 = np.array(coord1)
    coord2 = np.array(coord2)
    
    d = euclidian_distance(coord1, coord2)
    
    # No intersection if distance is greater than 2 times the radius or zero
    if d > 2 * radius or d == 0:
        return None
    
    radius_squared = radius**2
    a = d / 2
    h = math.sqrt(radius_squared - a**2)
    
    midpoint = (coord1 + coord2) / 2
    
    direction = (coord2 - coord1) / d
    
    perpendicular = np.array([-direction[1], direction[0]])
    
    intersection1 = midpoint + h * perpendicular
    intersection2 = midpoint - h * perpendicular
    
    # Calculating the midpoint
    intersection_midpoint = (intersection1 + intersection2) / 2
    
    return tuple(intersection_midpoint)

def check_intersections(current_position, current_vector, radius):
    """Get the points of intersection with other bots."""
    global robot_data
    global client_id
    intersections = []
    for robot_id, info in robot_data.items():
        if robot_id == client_id:
            continue
        other_position = info['position']
        other_vector = info['vector']
        points = calculate_intersection_points(current_position, other_position, radius)
        if points:
            intersections.append(points)
    return intersections

def check_border_intersection(current_position, radius, width, height):
    """Check if a circle intersects with the borders of the image."""
    x, y = current_position
    intersections = []
    # augments the detection radius for the border
    corrected_radius = radius + radius_correction
    
    # Check intersection with the left border (x = 0)
    if x < corrected_radius:
        intersections.append((0, y))
    
    # Check intersection with the right border (x = width)
    if x > width - corrected_radius:
        intersections.append((width, y))
    
    # Check intersection with the top border (y = 0)
    if y < corrected_radius:
        intersections.append((x, 0))
    
    # Check intersection with the bottom border (y = height)
    if y > height - corrected_radius:
        intersections.append((x, height))
    
    return intersections


def normalize_vector(vector):
    norm = np.linalg.norm(vector)
    if norm == 0:
        return vector  # Return the original vector if its norm is 0 (to avoid division by zero)
    return vector / norm

def avoid_collisions(current_position, normalized_vector, intersections, border_intersections):
    """Adjust movements to avoid collisions."""

    # Process intersections with other bots
    for point in intersections:
        vector_to_point = (point[0] - current_position[0], point[1] - current_position[1])
        normalized_vector_to_point = normalize_vector(vector_to_point)
        
        dot_product = np.dot(normalized_vector, normalized_vector_to_point)
        
        if dot_product < 0:
            print("Collision behind, move forward")
            MoveForward(0.3)
            return
        else:
            print("Collision in front, avoid to the side")
            # Decide to spin left or right based on cross product sign
            cross_product = np.cross(normalized_vector, normalized_vector_to_point)
            if cross_product > 0:
                #print("Avoid to the left")
                SpinLeft(0.3)
            else:
                #print("Avoid to the right")
                SpinRight(0.3)
            return
    
    # Process intersections with borders
    for border in border_intersections:
        vector_to_border = (border[0] - current_position[0], border[1] - current_position[1])
        normalized_vector_to_border = normalize_vector(vector_to_border)
        
        dot_product = np.dot(normalized_vector, normalized_vector_to_border)
        
        if dot_product < 0:
            print("Border behind, move forward")
            MoveForward(0.3)
            return
        else:
            print("Border in front, avoid to the side")
            # Decide to spin left or right based on cross product sign
            cross_product = np.cross(normalized_vector, normalized_vector_to_border)
            if cross_product > 0:
                print("Spin to the left to avoid border")
                SpinLeft(0.3)
            else:
                print("Spin to the right to avoid border")
                SpinRight(0.3)
            return

    print("No collisions detected")

def pathing_light():
    global robot_data
    global client_id
    global last_spin_time
    global notfinished
    global target_position  # New global to hold target position
    global reduced_avoidance_radius

    try:
        current_position = robot_data[client_id]['position']
        current_vector = robot_data[client_id]['vector']
    except KeyError:
        print(f"No key with that name: {client_id}")
        return
    intersections = check_intersections(current_position, current_vector, radius)
    border_intersections = check_border_intersection(current_position, radius, 1280, 720)

    if intersections or border_intersections:
        avoid_collisions(current_position, current_vector, intersections, border_intersections)
    elif target_position:  # If a target position is set, move towards it
        if not move_to_position(current_position, current_vector, target_position, target_tolerance):
            notfinished = False
            Stop()
            print(f"Arrived at target position: {target_position}")
        return
    else:
        current_time = robot.getTime()
        if current_time - last_spin_time >= 5:
            ldr_readings, distance_readings, angles = SpinTop(10, 0.1)
            highest_ldr_value = max(ldr_readings)
            highest_ldr_index = ldr_readings.index(highest_ldr_value)
            highest_ldr_angle = angles[highest_ldr_index]
            highest_ldr_distance = distance_readings[highest_ldr_index]
            print(highest_ldr_value)
            print(highest_ldr_angle)
            print(highest_ldr_distance)
            if highest_ldr_value > ldr_max_threshold:
                print("Found it!")
                #notfinished = False
                client.publish(topics['send'], f"foundit {current_position}")
                MoveBack(1)
            elif highest_ldr_value >= ldr_min_threshold:
                steer_to_angle(highest_ldr_angle)
                last_spin_time = current_time
            else:
                #print(f"LDR values too low ({ldr_min_threshold})")
                MoveForward(1)
        else:
            MoveForward(1)

def pathing_target():
    global robot_data
    global target_position
    global reduced_avoidance_radius
    global client_id
    global notfinished
    
    try:
        current_position = robot_data[client_id]['position']
        current_vector = robot_data[client_id]['vector']
    except KeyError:
        print(f"No key with that name: {client_id}")
        return
     # Calculate distance to target position
    distance_to_target = None
    reduction_factor = 1.0
    if target_position:
        distance_to_target = euclidian_distance(target_position, current_position)
        if distance_to_target < reduced_avoidance_radius:
            reduction_factor = distance_to_target / reduced_avoidance_radius

    print(f"Distance to target: {distance_to_target}, Reduction factor: {reduction_factor}")
    intersections = check_intersections(current_position, current_vector, reduced_radius)
    border_intersections = check_border_intersection(current_position, reduced_radius, 1280, 720)

    if intersections or border_intersections:
        avoid_collisions(current_position, current_vector, intersections, border_intersections)
    else:
        MoveForward(0.1)

def steer_to_vector(current_vector, target_vector):
    current_vector = normalize_vector(current_vector)
    target_vector = normalize_vector(target_vector)

    dot_product = np.dot(current_vector, target_vector)
    cross_product = np.cross(current_vector, target_vector)
    
    angle_radians = math.acos(dot_product)
    angle_degrees = math.degrees(angle_radians)

    max_turn_rate = 1.0
    min_turn_rate = 0.01
    speed = 0.2
    angle_radians = math.acos(dot_product)
    angle_degrees = math.degrees(angle_radians)

    turn_rate = min_turn_rate + (max_turn_rate - min_turn_rate) * (angle_degrees / 90.0)
    print(f"dot: {dot_product}")
    print(f"angle: {angle_degrees}")
    print(f"cross: {cross_product}")
    # If the dot product is close to 1, move forward
    if dot_product > 0.9659: # 0.995 5 degrees 15 0.9659
        MoveForward(speed)
    else:
        # Adjust direction
        if cross_product > 0:
            SpinRight(turn_rate)
            #MoveForward(speed)
        else:
            SpinLeft(turn_rate)
            #MoveForward(speed)

def move_to_position(current_position, current_vector, target_position, tolerance):
    target_x, target_y = target_position
    current_x, current_y = current_position
    
    distance = euclidian_distance(current_position, target_position)
    
    print(f"distance: {distance}")
    if distance <= tolerance:
        print("withing distance")
        return False  
     
    target_vector = (target_x - current_x, target_y - current_y)
    steer_to_vector(current_vector, target_vector)
    return True  

def steer_to_angle(target_angle):
    target_angle %= 360  # Normalize the target angle to 0-359 degrees

    if target_angle > 180:
        SpinRight((360 - target_angle) / 180)
    else:
        SpinLeft(target_angle / 180)
        
def on_message(client, userdata, msg):
    global topics
    global robot_data
    global client_id
    global notfinished
    global target_position  # New global to hold target position

    topic = msg.topic
    message = msg.payload.decode()

    if topic == f"robots/{client_id}/config":
        print("Received configuration response.")
        config = message.split(',')
        if len(config) == 2:
            topics['receive'] = config[0]
            topics['send'] = config[1]
            client.subscribe(topics['receive'])
            #print(f"Subscribed to {topics['receive']}")
            client.publish(topics['send'], f"{client_id} connected successfully")
            #print(f"Published '{client_id} connected successfully' to {topics['send']}")
        else:
            print("Invalid configuration format received.")

    elif topic == topics['receive']:
        print("Received message on the receive topic.")
        if message.startswith('foundit'):
            try:
                coordinates_str = message.replace("foundit {", "").replace("}", "")
                x_str, y_str = coordinates_str.split(',')
                x = int(x_str.strip())
                y = int(y_str.strip())
                print(f"Received coordinates: x={x}, y={y}")
                target_position = (x, y)  # Set the target position for the robot
                #notfinished = True  # Ensure pathing_light continues to run
            except ValueError:
                print("Error: Unable to parse coordinates from the received message.")
        else:
            try:
                data = json.loads(message)
                for robot_id, robot_info in data.items():
                    if robot_id != client_id:
                        if robot_id in robot_data:
                            robot_data[robot_id]['position'] = robot_info['position']
                            robot_data[robot_id]['vector'] = robot_info['vector']
                        else:
                            robot_data[robot_id] = {'position': robot_info['position'], 'vector': robot_info['vector']}
                    else:
                        robot_data[robot_id] = {'position': robot_info['position'], 'vector': robot_info['vector']}
                
                #print("Updated robot data:", robot_data)
                if notfinished:
                    pathing_light()
                    client.publish(topics['send'], f"request_positions")
            except json.JSONDecodeError:
                print(f"Received command: {message} on {topic}")
                handle_command(message)
    else:
        print(f"Received message on unknown topic {topic}: {message}")
        handle_command(message)
        
def handle_command(command):
    print("Received command:", command)
    command = command.strip().upper()
    if command == "MOVE_FORWARD":
        MoveForward(1)
    elif command == "MOVE_FORWARD_CONT":
        MoveForwardCont()
    elif command == "MOVE_BACK":
        MoveBack(1)
    elif command == "SPIN_LEFT":
        SpinLeft(1)
    elif command == "SPIN_RIGHT":
        SpinRight(1)
    elif command == "SPIN_TOP":
        SpinTop(10.0, 1)
    elif command == "START":
        started = True
    elif command == "stop":
        Stop()
    else:
        print("Invalid command.")

client = mqtt.Client(client_id)
client.on_message = on_message
def connect_mqtt():
    try:
        client.connect(broker, port)
        print("Robot connected to MQTT Broker!")
        client.subscribe(f"robots/{client_id}/config")
        client.subscribe(f"robots/positions")
        print(f"Subscribed to robots/{client_id}/config")
        client.publish(topic_register, client_id)
        print(f"Registration message sent: {client_id}")
    except Exception as e:
        print(f"Failed to connect to MQTT Broker: {e}")

def run_mqtt():
    print("Starting")
    connect_mqtt()
    message_thread = threading.Thread(target=lambda: client.loop_forever())
    message_thread.start()
    while robot.step(timestep) != -1:
        pass

run_mqtt()

