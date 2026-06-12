import socket
import network
import time
from umqtt.simple import MQTTClient
from machine import Pin, PWM, ADC
from poly_fit_3rd import polyfit3,eval_poly3

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


topics = [
        "chariot/#",
    ]

#       Reverse - Idle - Forward
# Left  3050    - 4900 - 6550
# Right 6550    - 4900 - 3050

# initial state definition
built_in_led = machine.Pin("LED", machine.Pin.OUT)
fled = Pin(FLED, Pin.OUT)
bled = Pin(BLED, Pin.OUT)
bled.value(False)
fled.value(False)

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
page = open("main.html", "r")
html = page.read()
page.close()

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
            print("\nConnected!\n")
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
    print(sta_if.ifconfig()[0]) # prints the IP on the serial

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


def move(motor, speed):
    print("Moving motor: ",motor," with speed: ",speed);
    if motor == "right":
        RightMotor.duty_u16(speed)
    elif motor == "left":
        LeftMotor.duty_u16(speed)

def stopMotors():
    print("Stopping motors")
    LeftMotor.duty_u16(4900)
    RightMotor.duty_u16(4900)

def test():
    print("Testing motors")

    move("left", 8200)
    move("right", 1500)

    duur = 5  # seconden
    start = time.time()

    leftCount = 0
    rightCount = 0

    prevLeft = Lsensor.value()
    prevRight = Rsensor.value()

    while time.time() - start < duur:

        currentLeft = Lsensor.value()
        currentRight = Rsensor.value()

        # Rising edge links
        if prevLeft == 0 and currentLeft == 1:
            leftCount += 1

        # Rising edge rechts
        if prevRight == 0 and currentRight == 1:
            rightCount += 1

        prevLeft = currentLeft
        prevRight = currentRight

        time.sleep(0.001)  # sneller samplen

    print("Left encoder pulses:", leftCount)
    print("Right encoder pulses:", rightCount)

    stopMotors()

def test_count(target_count=10):
    print(f"Driving until both encoders reach {target_count} pulses")

    leftCount = 0
    rightCount = 0

    prevLeft = Lsensor.value()
    prevRight = Rsensor.value()

    move("left", 7000)
    move("right", 2000)

    leftDone = False
    rightDone = False

    while not (leftDone and rightDone):

        currentLeft = Lsensor.value()
        currentRight = Rsensor.value()

        # Rising edge links
        if not leftDone and prevLeft == 0 and currentLeft == 1:
            leftCount += 1

            if leftCount >= target_count:
                move("left", 4900)   # stop linker motor
                leftDone = True
                print("Left target reached")

        # Rising edge rechts
        if not rightDone and prevRight == 0 and currentRight == 1:
            rightCount += 1

            if rightCount >= target_count:
                move("right", 4900)  # stop rechter motor
                rightDone = True
                print("Right target reached")

        prevLeft = currentLeft
        prevRight = currentRight

        time.sleep_ms(1)

    stopMotors()

    print("Finished")
    print("Left count :", leftCount)
    print("Right count:", rightCount)


#------------------------------------------------
def measure_rpm(sensor, duration=1.0):
    """Meet RPM via rising edges (10 slots per wheel)."""
    count = 0
    prev = sensor.value()

    start = time.ticks_ms()

    while time.ticks_diff(time.ticks_ms(), start) < duration * 1000:
        cur = sensor.value()

        if prev == 0 and cur == 1:
            count += 1

        prev = cur
        time.sleep_ms(1)

    rotations = count / 10  # 10 slots per wheel
    rpm = (rotations / duration) * 60
    return rpm


def find_pwm_for_rpm(
    motor,
    sensor,
    target_rpm,
    pwm_start=5000,
    step=200,
    tolerance=2.0,
    max_iter=15
):
    """
    Zoekt PWM die dicht bij target RPM komt.
    """

    pwm = pwm_start

    best_pwm = pwm
    best_error = 9999

    for i in range(max_iter):

        motor.duty_u16(pwm)
        time.sleep(1.0)  # stabilisatie

        rpm = measure_rpm(sensor, duration=1.0)

        error = target_rpm - rpm

        print(f"PWM={pwm}  RPM={rpm:.2f}  error={error:.2f}")

        # best match bewaren
        if abs(error) < abs(best_error):
            best_error = error
            best_pwm = pwm

        # goed genoeg?
        if abs(error) <= tolerance:
            break

        # richting bepalen
        if error > 0:
            pwm += step   # te langzaam → meer PWM
        else:
            pwm -= step   # te snel → minder PWM

        step = max(20, step // 2)  # steeds fijner zoeken

    motor.duty_u16(5000)  # stop

    print("\nBEST RESULT:")
    print("PWM:", best_pwm)
    print("RPM error:", best_error)

    return best_pwm
#------------------------------------------------




def mqtt_callback(topic, msg):
    print(
        "MQTT:",
        topic.decode(),
        "->",
        msg.decode()
    )

    if topic == b"chariot/move/right":
        move("right", int(msg.decode()))
    elif topic == b"chariot/move/left":
        move("left", int(msg.decode()))
    elif topic == b"chariot/move/forward":
        move("left", 6550)
        move("right", 3050)
    elif topic == b"chariot/move/reverse":
        move("left", 3050)
        move("right", 6550)
    elif topic == b"chariot/stop":
        stopMotors()
    elif topic == b"chariot/test":
        test_count()
    elif topic == b"chariot/calcpwm":
        print(find_pwm_for_rpm(
            LeftMotor,
            Lsensor,
            target_rpm=int(msg.decode())
        ))

        print(find_pwm_for_rpm(
            RightMotor,
            Rsensor,
            target_rpm=int(msg.decode())
        ))
    else:
        print("Received message on unknown topic:", topic.decode())

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



stopMotors()

connect_to_network()

mqtt_client = mqtt_connect_and_subscribe(
    broker_ip=mqtt_ip,
    username=mqtt_username,
    password=mqtt_password,
    port=mqtt_port
)


while True:
    mqtt_client.check_msg()
    time.sleep_ms(10)
#serve_pagina() # Blocking