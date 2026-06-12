####

# THIS CODE IS IN DEVELOPMENT AND DOES NOT SATISFY MOST REQUIREMENTS
# FOR ROBUSTNESS AND RELIABILITY
# USE IT AS BASE TO DEVELOP YOUR OWN!!!

###

# TODO: insert a 8 elements ring buffer for the IRQ calculated periods to use for the full PID

version="20260531"

'''
Useful starting point to setup the work environment
https://randomnerdtutorials.com/getting-started-raspberry-pi-pico-w/

This code exposes a small webpage allowing to tunr on or off the front (Red) and back (Green) leds and allowing to move
the two wheels forward (supposedly) for 1 second at moderate speed. To be fully tested.

Upload both this file main.html and poly_fit_3rd.py on the picoW
To test in your own house, change the ssid and password accordingly

13 May: added debouncing logic to IRQ function (separately for each channel)
20 May: streamilined the network credentials part using a separate credentials file
You may want to register your device MAC via
https://docs.datalabrotterdam.nl/services/pulsar-iot/tutorials/add-device
so you can connect to iotroam.
To get Mac address use "network_test.py"
31 May: Modified the approach to reading the encoders adding a time based interrupt that
samples the actual lines levels and  stores the period of the logic signal (from high to high)
of both encoders in "L_rising_period" and "R_rising_period". For usage see "diganose_encoders(..)"
TODO add some smart "rejects outlayers" method to get rid of obviously wrong values that are
either WAY too short (caused by noise) or WAY too long (caused by lost readings)

Added a function "run_at_speed(..)" that, using the encoders readings, attempts to keep
the wheels to the intended value for the duration of the move.

As of this code version it uses solely a proportional correction (the P in PID) but,
adding a "memory" in the encoders readers keeping track of several measured periods
a full PID can be implemented. Stability, due to some variability in encoders reading, must
be evaluated experimentally.
'''
print(f"Code Version: {version}")

import socket
import network
import time
from machine import Pin, PWM, ADC

from poly_fit_3rd import polyfit3,eval_poly3 # additional function for speed to RPM function

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

# assumes that a file is present with two lists of ssid(s) and pwd(s)
# ssid=['ssid_1','ssid_2','ssid_3']
# pwd=['pwd_1','pwd_2','pwd_3']

try:
    from credentials import ssid,pwd
except:
    print("no credentials file found. No connection will be possible")
    ssid=None
    pwd=None


# initial state definition
built_in_led = machine.Pin("LED", machine.Pin.OUT)
fled = Pin(FLED, Pin.OUT)
bled = Pin(BLED, Pin.OUT)
fled.value(True)
bled.value(False)
built_in_led.value(True)
time.sleep(1)
built_in_led.value(False)
time.sleep(1)
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

# fitting coefficient for the optimal cubic curve to provide a speed_to_PWM map for each wheel
# to be used in the 'set_wheel_speed(..)' function
coeff_LP = []
coeff_RP = []
coeff_LN = []
coeff_RN = []

# PID part
# as of now, only the Proportional coefficient is used
P=-0.0022
I=0 
D=0

# variable to indicate if network connection was succesful
is_connected=False

# START section of timer based interrupt polling the encoders lines

SAMPLE_PERIOD_US = 250 # samplig period in uS (1000==> 1 mS/1KHz and 250==> 0.25 mS/4 KHz) 

last_L_val=0
last_L_rising=0
last_L_falling=0
L_rising_period=0 # this (and the falling) are our useful values
L_falling_period=0
valid_L=False

last_R_val=0
last_R_time=0
last_R_rising=0
last_R_falling=0
R_rising_period=0 # this (and the falling) are our useful values
R_falling_period=0
valid_R=False

from machine import Timer

encoder_timer = Timer()

def get_encoders(timer):
    global last_L_val, last_R_val
    global last_L_rising, last_R_rising, last_L_falling, last_R_falling
    global L_rising_period, R_rising_period, L_falling_period, R_falling_period
    global valid_L, valid_R
    # read encoder pins here
    L_val=Lsensor.value()
    R_val=Rsensor.value()
    now=time.ticks_us()
    # check if a change has occurred 
    if(not L_val==last_L_val): # change has occurred in Left channel
        if L_val==1: # a rising transition
            L_rising_period=time.ticks_diff(now, last_L_rising)
            if last_L_rising>0:
                valid_L=True
            last_L_rising=now
        else: # then it is a falling transition
            L_falling_period=time.ticks_diff(now, last_L_falling)
            last_L_falling=now
        last_L_val=L_val
    if(not R_val==last_R_val): # change has occurred in Right channel
        if R_val==1: # a rising transition
            R_rising_period=time.ticks_diff(now, last_R_rising)
            if last_R_rising>0:
                valid_R=True
            last_R_rising=now
        else: # then it is a falling transition
            R_falling_period=time.ticks_diff(now, last_R_falling)
            last_R_falling=now
        last_R_val=R_val

def start_encoder_timer():
    encoder_timer.init(
    mode=Timer.PERIODIC,
    freq=1000000 // SAMPLE_PERIOD_US,
    callback=get_encoders
    )

def stop_encoder_timer():
    encoder_timer.deinit()
    
# END section of timer based interrupt polling the encoders lines


# loads the local page content
page = open("main.html", "r")
html = page.read()
page.close()


# START section where stability table is handled
stability_table={}
stability_table['L']={}
stability_table['R']={}

# has two "master" keys 'L' and 'R' and, for each key, a dict of values like {RPS,PWM}
# so to set use stability_table['L'][0.5]=4800 and to read stability_table['L'][0.5]
# use specific_RPS in stability_table['L'] to see if a value is available

def save_stability_table(D=stability_table, filename="stability_table.txt"):
    with open(filename, "w") as f:
        for side in D:
            for key in D[side]:
                f.write("{},{},{}\n".format(
                    side,
                    key,
                    D[side][key]
                ))

def load_stability_table(filename="stability_table.txt"):
    D = {"L": {}, "R": {}}

    with open(filename, "r") as f:
        for line in f:
            line = line.strip()

            if not line:
                continue

            side, key, value = line.split(",")

            D[side][float(key)] = int(value)

    return D

# try loading existing (if any) stabulity table or use default empty one if fails
try:
    stability_table=load_stability_table()
    print("Loaded stability table from file")
    print("stability_table: ",stability_table)
except:
    print("No stability table in file")
    pass # table is already prepared

# END section where stability table is saved


# START Networking section

# activate the Pico Lan
# you will need to register the device on the network.
# use the "network_test.py" to get your device Mac address

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

# END Networking session
        

def calculate_PWM(RPS_L=0,RPS_R=0):
    global coeff_LP, coeff_RP, coeff_LN, coeff_RN
    if coeff_LP==[] or coeff_RP==[]:
        print("Perform autocalibration first")
        return(-1)
    if RPS_L>0:
        PWM_L=int(eval_poly3(RPS_L,coeff_LP)*2000+5000)
    else:
        PWM_L=int(eval_poly3(-RPS_L,coeff_LN)*2000+5000)
    if RPS_R>0:
        PWM_R=int(eval_poly3(RPS_R,coeff_RP)*2000+5000)
    else:
        PWM_R=int(eval_poly3(-RPS_R,coeff_RN)*2000+5000)
        
    return(PWM_L,PWM_R)
    
def load_calibration():
    # uses coefficients stored in wheel_calibration.py
    # overwrites those with new calibration.
    # fallsback to default if no calibration file is present.
    # there are no calibration functions in this file as the calibration has been
    # replaced by a  stability table that improves automatically based on the system use
    global coeff_LP,coeff_RP,coeff_LN,coeff_RN
    try:
        from wheels_calibration import coeff_LP, coeff_LN, coeff_RP, coeff_RN, default_values
        if default_values:
            print("Loaded DEFAULT calibration coefficients. Please run a calibration")
        else:
            print("Loaded device specific calibration coefficients.")
            print("Recalibration may still be needed from time to time")
    except:
        print("Missing calibration file: fallback on default values")
        coeff_LP=(-0.1777802, 0.6523898, -0.1553434, 0.08150997)
        coeff_RP=(0.0814823, -0.4187451, 0.08430676, -0.1871835)
        coeff_LN=(0.2038952, -0.6254874, 0.03229254, -0.1854067)
        coeff_RN=(0.04240555, 0.0646009, 0.2027688, -0.006709341)    
    '''
    results_L=calibrate(5200,6600,200, 'Left')
    results_R=calibrate(4600,3200,-200,'Right')

    # results_L=[[5200, 0.4350753], [5400, 0.7238501], [5600, 0.9432401], [5800, 1.134747], [6000, 1.312374], [6200, 1.485946], [6400, 1.627926]]
    # results_R=[[4600, 0.3048474], [4400, 0.6584736], [4200, 0.8859709], [4000, 1.102765], [3800, 1.274819], [3600, 1.445861], [3400, 1.575675]]
    '''

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
                if str(line).find("PRESS_8=TESTL") !=-1:
                    while True:
                        print(Lsensor.value())
                if str(line).find("PRESS_9=TESTR") !=-1:
                    while True:
                        print(Rsensor.value())
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


def diagnose_encoder(RPS_L=1,RPS_R=1,t_test=5):
    ''' same as above, but this time we rests on a timer based interrupt
        techically, this is not blocking
    '''
    global L_rising_period, R_rising_period
    #global last_L_rising, last_R_rising
    #global L_falling_period, R_falling_period
    global events_L, events_R
    # TODO define the intial pins status
    last_L_val=Lsensor.value()
    last_R_val=Rsensor.value()
    # sets initial values
    events_L=[0]
    events_R=[0]
    
    # calculates the bona-fide PWM for the motors
    PWM_L,PWM_R=calculate_PWM(RPS_L,RPS_R)

    # to define test length
    now=time.ticks_us() # current time

    # starts the motor
    LeftMotor.duty_u16(PWM_L)
    RightMotor.duty_u16(PWM_R)
    
    # enables the interrupt
    start_encoder_timer()
    
    # starts monitoring and saving the periods
    # this is a bit blocking but it is mostly diagnostic
    while(time.ticks_us()-now<t_test*1000000):
        # cheks the rising periods (left and right) and as soon as they change
        # adds them to the events_XX array
        # left channel
        if (not L_rising_period==events_L[-1]): # it could miss a period if 2 were to coincide
            events_L.append(L_rising_period)
        # right channel
        if (not R_rising_period==events_R[-1]):
            events_R.append(R_rising_period)
   
    # stops the interrupt
    stop_encoder_timer()
    
    # stops motors
    LeftMotor.duty_u16(5000)
    RightMotor.duty_u16(5000)

    # removes the first "dummy" event and the second one that is meaningless 
    del events_L[0:2]
    del events_R[0:2]
    print("Done, results are in events_L[] and events_R[]")
    print("Left channel results:")
    for result in events_L:
        print(f"{result}")

    print("right channel results:")
    for result in events_R:
        print(f"{result}")
        
    
def run_at_speed(RPS_L=1,RPS_R=1,t_test=5, diagnostic=False):
    ''' using the signal from the encoders sets the wheel speed to the desired value
        as there are 10 slots, the target period will be 1/RPS/10*1000000 in uS
        e.g. for RPS=1 Target Period is 100000 uS
    '''
    global L_rising_period, R_rising_period
    global valid_L,valid_R
    
    # PID parameters 
    # errors are in the order of 1000 to 5000 (1 to 5%) where positive error means too fast
    # PWM variations should be in the order of 12 to 60 (12 for 1%, 60 for 5%) 
    global P,I,D
    # theoretical value is obtained as -60/5000=-0.012 and then tweaked manually

    # define the intial pins status
    last_L_val=Lsensor.value()
    last_R_val=Rsensor.value()
    # sets two local variables for the last recored periods
    last_L_period=0
    last_R_period=0
    # here are the targets for the periods
    target_L_period=1/RPS_L/10*1000000
    target_R_period=1/RPS_R/10*1000000
    if diagnostic: print(f"targets: L={target_L_period} R={target_R_period}")
    # variables to see if stability was achieved for multiple readings
    # and, if so, allows to insert the corrected PWM value in a reference table
    stability_L=False
    stability_R=False
    stability_count_required=5 # if after this number of period we did not have to correct it is stable enough
    stability_threshold=0.02 # this defined the "good enough" to avoid corrections
    stability_table_updated=False
    
    global stability_table # here are conserved the values for which we achieved a stable PWM
    
    # calculates the bona-fide PWM for the motors from the hard-coded calibration curve
    PWM_L,PWM_R=calculate_PWM(RPS_L,RPS_R)

    # check if there is a corresponding entry in the stability table and use it if present
    if RPS_L in stability_table['L']:
        PWM_L=stability_table['L'][RPS_L]
        if diagnostic: print(f"Updated PWM_L from stability table to {PWM_L}")
    if RPS_R in stability_table['R']:
        PWM_R=stability_table['R'][RPS_R]
        if diagnostic: print(f"Updated PWM_R from stability table to {PWM_R}")

    # two variables used in the PID (for now only P) controller
    corrected_L=PWM_L
    corrected_R=PWM_R

    # to define test length
    now=time.ticks_us() # current time

    # starts the motor
    LeftMotor.duty_u16(PWM_L)
    RightMotor.duty_u16(PWM_R)
        
    # enables the interrupt
    start_encoder_timer()

    # wait a bitfor values stabilization
    time.sleep_ms(500)
    # starts monitoring and CORRECTING PWM values
    # this is a bit blocking but can be moved to IRQ later
    while(time.ticks_us()-now<t_test*1000000):
        # cheks the rising periods (left and right) and as soon as they change
        # applies a PWM correction
        # left channel
        if (not L_rising_period==last_L_period)and valid_L: # it could miss a period if 2 were to coincide
            # changes PWM with P (and then ID)
            error_L=target_L_period-L_rising_period # if positive, we are running too fast
            if abs(error_L/target_L_period)<stability_threshold: # do not correct if good enough
                stability_L+=1
                #print("stability: ", error_L/target_L_period, corrected_L)
            else:
                corrected_L=int(PWM_L+error_L*P)
                stability_L=0 # we start from zero again if large errors had to be corrected
                LeftMotor.duty_u16(corrected_L)
            if diagnostic : print("L ",L_rising_period,corrected_L,100*error_L/target_L_period,stability_L>=stability_count_required)
            last_L_period=L_rising_period
            # verify if we achieved a stable configuration
            if stability_L>=stability_count_required and corrected_L>0:
                # save into a dict table to be used when setting the PWM
                stability_table['L'][RPS_L]=corrected_L
                stability_table_updated=True

        # right channel (same as left with relevant variables)
        if (not R_rising_period==last_R_period) and valid_R:
            # change PWM
            error_R=target_R_period-R_rising_period # if positive, we are running too fast
            if abs(error_R/target_R_period)<stability_threshold: # do not correct if good enough
                stability_R+=1
            else:
                corrected_R=int(PWM_R-error_R*P) # minus as the PWM decreases for the right motor to make it faster
                stability_R=0
                RightMotor.duty_u16(corrected_R)
            if diagnostic: print("R ",R_rising_period,corrected_R,100*error_R/target_R_period,stability_R>=stability_count_required)
            last_R_period=R_rising_period
            # verify if we achieved a stable configuration
            if stability_R>=stability_count_required and corrected_R>0:
                # save into a dict table to be used when setting the PWM
                stability_table['R'][RPS_R]=corrected_R
                stability_table_updated=True

    # stops the interrupt
    stop_encoder_timer()
    
    # stops motors
    LeftMotor.duty_u16(5000)
    RightMotor.duty_u16(5000)
    
    # saves new stability_table if changes occurred
    if stability_table_updated:
        save_stability_table(stability_table)
        stability_table_updated=False

def run_steps(steps_L,steps_R,RPS_L=1,RPS_R=1):
    ''' 
        The function allows to rotate both wheels (or one) for a set number of encoder
        steps (10 for a full rotation) at two separate RPS values (defaulting at 1)
        We count the steps in the encoder using the change events of
        the L_rising_period and R_rising_period
    '''
    global L_rising_period, R_rising_period
    
    # sets two local variables for the last recored periods
    last_L_period=0
    last_R_period=0
    
    # elapsed steps
    elapsed_steps_L=0
    elapsed_steps_R=0
    
    # calculates the bona-fide PWM for the motors
    # does not use PID correction 
    PWM_L,PWM_R=calculate_PWM(RPS_L,RPS_R)

    # starts the motor
    LeftMotor.duty_u16(PWM_L)
    RightMotor.duty_u16(PWM_R)

    # enables the interrupt
    start_encoder_timer()

    # starts monitoring and CORRECTING PWM values
    # this is a bit blocking but can be moved to IRQ later
    while(steps_L>0 or steps_R>0):
        if (not L_rising_period==last_L_period): # it could miss a period if 2 were to coincide
            # decrements steps_L until it gets to zsro
            steps_L-=1
            last_L_period=L_rising_period
            if steps_L<=0: # stops the motor
                LeftMotor.duty_u16(5000)
                
        if (not R_rising_period==last_R_period): # it could miss a period if 2 were to coincide
            # decrements steps_R until it gets to zsro
            steps_R-=1
            last_R_period=R_rising_period
            if steps_R<=0: # stops the motor
                RightMotor.duty_u16(5000)        

    
## ENTRY POINT FOR MAIN CODE

# load calibration file for motors if present, fallback to a a default if not present
# the fallback is good enough to define initial PWM values
load_calibration()
# try connecting to network and, if ok, start listening to port 80
# global variable is_connected contains network connection status
connect_to_network()

 
# tries to serve the web page (and responds to API calls) for control
# process is blocking (can be improved with threading)
serve_pagina()



# test
#save_capture_csv()
#diagnose_encoder_no_IRQ()
'''
diagnose_encoder()
#events_R.sort()
#events_L.sort()
avg_L=sum(events_L)/len(events_L)
avg_R=sum(events_R)/len(events_R)
delta_L=max(events_L)-min(events_L)
delta_R=max(events_R)-min(events_R)
print(f"Left encoder: average period {avg_L} uS with variation of {delta_L}")
print(f"Right encoder: average period {avg_R} uS with variation of {delta_R}")
'''
#run_at_speed(0.5,0.5,5)
#run_steps(100,100)
#print("execute 'run_at_speed(1,1,5)' to move for 5 seconds forward")



