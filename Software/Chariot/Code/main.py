import socket
import network
import time
from umqtt.simple import MQTTClient
from machine import Pin, PWM, ADC
from robot_config import ROBOTS
#from poly_fit_3rd import polyfit3,eval_poly3



BUILT_IN_LED=25 # Built in led
FLED=20 # Front led Red
BLED=21 # Back led Green
PWM_LM=6 # Left Continuous Servo
PWM_RM=7 # Right Continuous Servo
PWM_SC=10 # Panning Servo
SDA=4
SCL=5
MISO=16
MOSI=19
SCK=18
CS=17

# added May 2026 - 2 more pins used for the Left and Right IR barrier encoders
EN_R=11 # GPIO14=Pin 19==> Right Connector Pin 2 (looking from the top - back side of charior)
EN_L=14 # GPIO11=Pin 15==> Right Connectot Pin 5 (looking from the top - back side of charior)

try:
    from credentials import ssid,pwd,mqtt_ip,mqtt_port,mqtt_username,mqtt_password
except:
    print("no credentials file found. No connection will be possible")
    ssid=None
    pwd=None


#       Reverse - Idle - Forward
# Left  3050    - 4900 - 6550
# Right 6550    - 4900 - 3050

# initial state definition
built_in_led = machine.Pin("LED", machine.Pin.OUT)
fled = Pin(FLED, Pin.OUT)
bled = Pin(BLED, Pin.OUT)
bled.value(False)
fled.value(False)

# connection status
# 0 = not connected 
# 1 = requested connection
# 2 = connecting
# 3 = connected
state = 0

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

# loads the local page content
# page = open("main.html", "r")
# html = page.read()
# page.close()


def connect_to_network():
    global is_connected
    global ssid,pwd
    if not ssid:
        print("No network credentials available. Unable to setup a connection")
        return
    network.hostname("mypicow") #wlan.config(hostname="mypico")
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    print("Hostname set to: "+str(network.hostname()))
    
    time0=time.time()
    wlan.connect(ssid, pwd)
    while 1:
        if(wlan.isconnected()):
            is_connected=True
            print("\nConnected to "+str(ssid)+"!\n")
            built_in_led.value(True)
            break
        else:
            print(".")
            is_connected=False
            time.sleep(1)
            if(time.time()-time0>10):
                print("Connection could not be established")
                break
    sta_if = network.WLAN(network.STA_IF)
    mac = wlan.config('mac')
    global mac_str
    mac_str = ':'.join('{:02X}'.format(b) for b in mac)
    print("MAC address:", mac_str)

    print("IP address:", sta_if.ifconfig()[0]) # prints the IP on the serial

    global s # the socket...
    
    if not is_connected:
        print("Device not network enabled: no web server will be activated")
        return
    # listen on port 80
    addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(addr)
    print("Listening to port 80\n")
    s.listen(1)
    fled.value(True)

def serve_pagina():
    global s # the socket
    global is_connected # connection status
    while (is_connected==True):
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


# PWM signals  Reverse - Idle - Forward
# Left 	         3050  - 4900 - 6550
# Right          6550  - 4900 - 3050


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

    if command == "SS":
        stopMotors()
        return

    if command not in MOTOR_CFG:
        print("Unknown command:", command)
        return

    left_pwm, right_pwm = MOTOR_CFG[command]

    moveMotor("left", left_pwm)
    moveMotor("right", right_pwm)


#------------------------------------------------
#------------------------------------------------


def mqtt_callback(topic, msg):
    topic = topic.decode()
    msg = msg.decode()
    global state
    print("MQTT:", topic, "->", msg)

    if topic == f"Robots/Control/{mac_str}/Status":
        if msg == "checking":
            print("Request received... waiting for connection to MQTT broker")
            # Led op geel
            state = 2 # connecting
            time.sleep(0.1)

        elif msg == "connected":
            print("Connection to MQTT broker established")
            # Led op groen
            state = 3 # connected
            time.sleep(0.1)

        else:
            print("Unknown status message received:", msg)


    elif topic == f"Robots/Control/{mac_str}/Commands":
        # if state == 3:  # only process commands if connected
            move(msg)
        # else:
        #     print("Received command but not connected to MQTT broker. Ignoring command.")


    # ----- Test topics -----
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
        port= 1883,
        client_id="pico_client",
    ):

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

    bled.value(True)
    return client


#------------------------------------------------
#                   Init
#------------------------------------------------

stopMotors()
connect_to_network()

if mac_str in ROBOTS:
    MOTOR_CFG = ROBOTS[mac_str]
    print("Loaded motor config for", mac_str)
else:
    raise Exception("No motor configuration found for MAC: " + mac_str)

topics = [
    "chariot/#",
    f"Robots/Control/{mac_str}/Status",
    f"Robots/Control/{mac_str}/Commands"
]

mqtt_client = mqtt_connect_and_subscribe( # Connect to MQTT broker and subscribe to topics
    broker_ip=mqtt_ip,
    username=mqtt_username,
    password=mqtt_password,
    port=mqtt_port
)

mqtt_client.publish( # Publish mac adress to the broker to indicate that this device is connecting
    "Robots/Control/Connecting",
    mac_str
)
print("Requested connection to MQTT broker with client ID:", mac_str)
state = 1

#------------------------------------------------
#                   Main loop
#------------------------------------------------

while True:
    mqtt_client.check_msg()
    time.sleep_ms(10)
