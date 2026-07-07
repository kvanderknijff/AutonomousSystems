import socket
import network
import time
import machine
from umqtt.simple import MQTTClient
from machine import Pin, PWM, ADC
from robot_config import ROBOTS
from nav_goal import (
    GoalNavigator,
    GOAL_ACTION_CLEAR,
    parse_config_payload,
    parse_goal_payload,
    parse_position_payload,
    format_report_payload,
)
#from poly_fit_3rd import polyfit3,eval_poly3

DISCONNECT_TIMEOUT=3000 # milliseconds
BUILT_IN_LED=25 # Built in led
FLED=20 # Front led Red
BLED=21 # Back led Green
PWM_LM=6 # Left Continuous Servo
PWM_RM=7 # Right Continuous Servo
PWM_SC=10 # Panning Servo
# SDA=4
# SCL=5
# MISO=16
# MOSI=19
# SCK=18
# CS=17
GREEN_LED_PIN = 18
BLUE_LED_PIN = 19

# added May 2026 - 2 more pins used for the Left and Right IR barrier encoders
EN_R=11 # GPIO14=Pin 19==> Right Connector Pin 2 (looking from the top - back side of charior)
EN_L=14 # GPIO11=Pin 15==> Right Connectot Pin 5 (looking from the top - back side of charior)

try:
    from credentials import ssid,pwd,mqtt_ip,mqtt_port,mqtt_username,mqtt_password
except:
    print("no credentials file found. No connection will be possible")
    ssid=None
    pwd=None


# initial state definition
built_in_led = machine.Pin("LED", machine.Pin.OUT)
fled = Pin(FLED, Pin.OUT)
bled = Pin(BLED, Pin.OUT)
green_led = Pin(GREEN_LED_PIN, Pin.OUT)
blue_led = Pin(BLUE_LED_PIN, Pin.OUT)
green_led.value(False)
blue_led.value(False)
bled.value(False)
fled.value(True)


# connection status
# 0 = not connected 
# 1 = requested connection
# 2 = connecting
# 3 = connected
state = 0
last_connect_request = 0
robot_aruco_id = None
navigator = GoalNavigator(turn_pulse_sec=0.15, turn_settle_sec=0.35)
last_applied_command = None
navigation_topics_ready = False

#setus up servos
LeftMotor = PWM(Pin(PWM_LM))
LeftMotor.freq(50)
RightMotor = PWM(Pin(PWM_RM))
RightMotor.freq(50)
PanMotor = PWM(Pin(PWM_SC))
PanMotor.freq(50)

# setup encoder sensor
Lsensor = machine.Pin(EN_L, machine.Pin.IN, machine.Pin.PULL_DOWN)
Rsensor = machine.Pin(EN_R, machine.Pin.IN, machine.Pin.PULL_DOWN)

#------------------------------------------------
#               Network
#------------------------------------------------


def connect_to_network():
    global wifi_connected
    global mac_str
    global ssid, pwd

    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    print("Connecting to WiFi network:", ssid)

    wlan.connect(ssid, pwd)

    start_time = time.time()

    while not wlan.isconnected():

        print(".", end="")
        time.sleep(1)

        if time.time() - start_time > 10:
            print("\nConnection timeout")
            wifi_connected = False
            return False

    wifi_connected = True
    print("Connected!")
    mac = wlan.config('mac')
    mac_str = ':'.join('{:02X}'.format(b) for b in mac)
    print("MAC address:", mac_str)
    print("IP address :", wlan.ifconfig()[0])
    built_in_led.value(True)

    return True


def serve_pagina():
    global s # the socket
    global wifi_connected # connection status
    while (wifi_connected==True):
        cl, addr = s.accept()
        print("Incoming connection request from: "+str(addr)+"\n")
        # here is the place where we get the request body...
        cl_file = cl.makefile('rwb', 0)
        found=False    
        while True:
            line = cl_file.readline()
            if not line or line == b'\r\n':
               break
            if not found: 
                if str(line).find("?PRESS=FRONT_LED_ON") !=-1:
                    #print("Command_1 ON received")
                    fled.value(True)
                    found=True
                if str(line).find("?PRESS_1=FRONT_LED_OFF") !=-1:
                    #print("Command_1 OFF received")
                    fled.value(False)
                    found=True
                if str(line).find("?PRESS_2=BACK_LED_ON") !=-1:
                    #print("Command_2 ON received")
                    bled.value(True)
                    found=True
                if str(line).find("?PRESS_3=BACK_LED_OFF") !=-1:
                    #print("Command_2 OFF received")
                    bled.value(False)
                    found=True
                if str(line).find("PRESS_4=MOVE") !=-1:
                    #print("Command MOVE received: ",str(line))
                    #/?speed=1.5&PRESS_4=MOVE
                    speed_Ls=str(line).split('&')[0]
                    speed_Rs=str(line).split('&')[1]
                    time_s=str(line).split('&')[2]
                    speed_L=float(speed_Ls.split('speed_L=')[-1])
                    speed_R=float(speed_Rs.split('speed_R=')[-1])
                    time_float=float(time_s.split('time=')[-1])
                    print(f"setting speed_L={speed_L} speed_R={speed_R} time={time_float}")
                    run_at_speed(speed_L,speed_R,time_float, diagnostic=True)
                    #MoveForward(25,5)
                    found=True
                if str(line).find("PRESS_6=MOVE_STEPS") !=-1:
                    #print("Command MOVE received: ",str(line))
                    #/?speed=1.5&PRESS_4=MOVE
                    steps_Rs=str(line).split('&')[0]
                    steps_Ls=str(line).split('&')[1]
                    speed_Rs=str(line).split('&')[2]
                    speed_Ls=str(line).split('&')[3]
                    steps_R=float(steps_Rs.split('steps_Rs=')[-1])
                    steps_L=float(steps_Ls.split('steps_Ls=')[-1])
                    speed_R=float(speed_Rs.split('speed_Rs=')[-1])
                    speed_L=float(speed_Ls.split('speed_Ls=')[-1])
                    print(f"setting steps L={steps_L}, steps R={steps_R}, speed L={speed_L}, speed R={speed_R}")
                    run_steps(steps_L,steps_R,speed_L,speed_R)
                    #MoveForward(25,5)
                    found=True
                if str(line).find("PRESS_7=DIAGNOSE") !=-1:
                    #print("Command MOVE received: ",str(line))
                    #/?speed=1.5&PRESS_4=MOVE
                    diagnose_encoder()
                    #MoveForward(25,5)
                    found=True
                if str(line).find("?PRESS_5=CALIBRATE") !=-1:
                    print("Command CALIBRATE received")
                    #autocalibrate()
                    found=True       
        # we process the response file, We can add placeholders to turn change the page aspect
        response=html # default page, placeholders needs to be replaced before submitting
        # send the page
        cl.send('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
        cl.send(response)
        cl.close()
    else:
        print("No connection is present. No web pagina is being server")


def check_server_status():
    try:
        addr = socket.getaddrinfo(mqtt_ip, 8080)[0][-1]
        s = socket.socket()
        s.settimeout(5)
        s.connect(addr)
        # print("Verbinding gelukt")
        s.close()

    except Exception as e:
        print("SERVER IS NOT RUNNING!!!")


#------------------------------------------------
#               Motor control
#------------------------------------------------


def moveMotor(motor, speed):
    print("Moving motor: ",motor," with speed: ",speed);
    if motor == "right":
        RightMotor.duty_u16(speed)
    elif motor == "left":
        LeftMotor.duty_u16(speed)

def stopMotors():
    print("Stopping motors")
    LeftMotor.duty_u16(4900)
    RightMotor.duty_u16(4900)


def move(command):
    global last_applied_command
    if state != 3:
        print("Received command but not connected to MQTT broker. Ignoring command.")
        return

    if command == "SS":
        print("SS received")
        stopMotors()
        last_applied_command = "SS"
        return

    if command not in MOTOR_CFG:
        print("Unknown command:", command)
        return

    left_pwm, right_pwm = MOTOR_CFG[command]
    last_applied_command = command
    moveMotor("left", left_pwm)
    moveMotor("right", right_pwm)


def parse_bracket_payload(payload):
    text = payload.strip()
    if text.startswith("[") and text.endswith("]"):
        return text[1:-1]
    return text


def publish_report(status, seq=0):
    global mqtt_client
    if mqtt_client is None or state != 3:
        return
    x = navigator.x
    y = navigator.y
    payload = format_report_payload(status, seq=seq, x=x, y=y)
    topic = f"Robots/Data/{mac_str}/Report"
    mqtt_client.publish(topic, payload)
    print("Report:", topic, payload)


def subscribe_navigation_topics(client):
    global navigation_topics_ready
    topics = [
        f"Robots/Data/{mac_str}/Goals",
        f"Robots/Data/{mac_str}/Config",
        "Robots/Data/Positions/Physical",
        "Robots/Data/Positions",
    ]
    for topic in topics:
        client.subscribe(topic)
        print("Subscribed to:", topic)
    navigation_topics_ready = True


def handle_goal_message(msg):
    goal = parse_goal_payload(msg)
    if goal is None:
        print("Ignored invalid goal:", msg)
        return
    if goal["action"] == GOAL_ACTION_CLEAR:
        navigator.clear_goal()
        print("Goal cleared")
        return
    navigator.set_goal(
        goal["target_x"],
        goal["target_y"],
        tolerance=goal.get("tolerance"),
        seq=goal.get("seq", 0),
    )
    print(
        "Goal set:",
        goal["target_x"],
        goal["target_y"],
        "tol",
        goal.get("tolerance"),
    )


def handle_config_message(msg):
    global robot_aruco_id
    config = parse_config_payload(msg)
    if config is None:
        return
    robot_aruco_id = config["aruco_id"]
    navigator.set_aruco_id(robot_aruco_id)
    print("Assigned ArUco ID:", robot_aruco_id)


def handle_position_message(msg):
    position = parse_position_payload(msg)
    if position is None:
        return
    if position.get("kind") == "corner":
        navigator.update_field_corner(
            position["aruco_id"],
            position["x"],
            position["y"],
        )
        return
    navigator.update_fleet_position(
        position["aruco_id"],
        position["x"],
        position["y"],
    )
    if robot_aruco_id is not None and position["aruco_id"] != robot_aruco_id:
        return
    navigator.update_position(
        position["x"],
        position["y"],
        position["orientation"],
    )


def run_navigation_tick():
    global last_applied_command
    if state != 3:
        return

    result = navigator.tick()
    if result is None:
        return

    if len(result) == 3:
        kind, value, seq = result
    else:
        kind, value = result
        seq = navigator.seq

    print("value:", value, "kind:", kind, "seq:", seq)

    if kind == "command":
        if value != last_applied_command or value == "SS":
            move(value)
            last_applied_command = value
    elif kind == "report":
        publish_report(value, seq=seq)
        move("SS")
        last_applied_command = "SS"


#------------------------------------------------
#               MQTT handling
#------------------------------------------------

def mqtt_callback(topic, msg):
    topic = topic.decode()
    msg = msg.decode()
    global state, navigation_topics_ready
    # print("MQTT:", topic, "->", msg)

    if topic == f"Robots/Control/{mac_str}/Status":
        if msg == "[checking]":
            if state != 1: return
            print("Request received... waiting for connection to server")
            blue_led.value(True)
            state = 2 # connecting
            time.sleep(0.1)

        elif msg == "[connected]":
            if state != 2: return
            print("Connection to server established")
            blue_led.value(False)
            green_led.value(True)
            state = 3 # connected
            if not navigation_topics_ready:
                subscribe_navigation_topics(mqtt_client)
            time.sleep(0.1)

        elif msg == "[disconnected]":
            print("Connection to server lost")
            stopMotors()
            navigator.clear_goal()
            navigation_topics_ready = False
            blue_led.value(False)
            green_led.value(False)
            state = 1 # disconnected

        else:
            print("Unknown status message received:", msg)

    elif topic == f"Robots/Data/{mac_str}/Commands":
        print("Received move command:", msg)
        navigator.enter_manual_mode()
        move(parse_bracket_payload(msg))

    elif topic == f"Robots/Data/{mac_str}/Goals":
        handle_goal_message(msg)

    elif topic == f"Robots/Data/{mac_str}/Config":
        handle_config_message(msg)

    elif topic in ("Robots/Data/Positions", "Robots/Data/Positions/Physical"):
        handle_position_message(msg)

    elif topic == "chariot/move/right":
        moveMotor("right", int(msg))
    elif topic == "chariot/move/left":
        moveMotor("left", int(msg))
    elif topic == "chariot/move/forward":
        moveMotor("left", 6550)
        moveMotor("right", 3050)
    elif topic == "chariot/move/reverse":
        moveMotor("left", 3050)
        moveMotor("right", 6550)
    elif topic == "chariot/stop":
        stopMotors()
    elif topic == "chariot/test":
        print("Left")
        print(pwm_to_rps_map("left", step=50))
        print("Right")
        print(pwm_to_rps_map("right", step=50))
    else:
        print("Received message on unknown topic:", topic)


def mqtt_connect_and_subscribe(
        broker_ip,
        username,
        password,
        port=1883,
        client_id="pico_client",
    ):

    global mqtt_connected

    try:
        client = MQTTClient(
            client_id=client_id,
            server=broker_ip,
            port=port,
            user=username,
            password=password
        )

        client.set_callback(mqtt_callback)

        print("Connecting to MQTT broker...")
        client.connect()

        print("Connected")

        for topic in topics:
            client.subscribe(topic)
            print("Subscribed to:", topic)

        mqtt_connected = True
        return client

    except Exception as e:
        mqtt_connected = False
        print("MQTT connection failed:", e)
        return None


#------------------------------------------------
#                   Init
#------------------------------------------------
print("------------------------------------------------------------")
stopMotors()
connect_to_network()

if not wifi_connected:
    print("Could not connect to WiFi network. Please check your credentials.")
    machine.reset()
print("------------------------------------------------------------")


if mac_str in ROBOTS:
    MOTOR_CFG = ROBOTS[mac_str]
    print("Loaded motor config for", mac_str)
else:
    print("No motor configuration found for MAC: " + mac_str)
print("------------------------------------------------------------")

topics = [
    "chariot/#",
    f"Robots/Control/{mac_str}/Status",
    f"Robots/Data/{mac_str}/Commands",
]

mqtt_client = mqtt_connect_and_subscribe( # Connect to MQTT broker and subscribe to topics
    client_id = mac_str,
    broker_ip=mqtt_ip,
    username=mqtt_username,
    password=mqtt_password,
    port=mqtt_port
)

if not mqtt_connected:
    print("Could not connect to MQTT broker. Please check your credentials and broker status.")
    bled.value(False)
    while True:
        fled.value(not fled.value())
        time.sleep_ms(300)  # Stay in a loop if MQTT connection fails

print("------------------------------------------------------------")

# Connected to WiFi and MQTT broker, now indicate that the device is ready
state = 1
fled.value(False)


#------------------------------------------------
#                   Main loop
#------------------------------------------------


while True:
    time.sleep_ms(10)
    mqtt_client.check_msg()
    run_navigation_tick()
    if state == 1:
        now = time.ticks_ms()
        if time.ticks_diff(now, last_connect_request) >= 10000:
            check_server_status()
            mqtt_client.publish("Robots/Control/Connecting", mac_str)
            print("Requested connection to server with client ID:", mac_str)
            last_connect_request = now
            bled.value(True)
            time.sleep_ms(200)
            bled.value(False)