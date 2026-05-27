####

# THIS CODE IS IN DEVELOPMENT AND DOES NOT SATISFY MOST REQUIREMENTS
# FOR ROBUSTNESS AND RELIABILITY
# USE IT AS BASE TO DEVELOP YOUR OWN!!!

###

'''
Useful starting point to setup the work environment
https://randomnerdtutorials.com/getting-started-raspberry-pi-pico-w/

This code exposes a small webpage allowing to tunr on or off the front (Red) and back (Green) leds and allowing to move
the two wheels forward (supposedly) for 1 second at moderate speed. To be fully tested.

Upload both this file main.html and poly_fit_3rd.py on the picoW
To test in your own house, change the ssid and password accordingly

13 May: added debouncing logic to IRQ function (separately for each channel)
20 May: streamilined the network credentials part using a separate
You may want to register your device MAC via
https://docs.datalabrotterdam.nl/services/pulsar-iot/tutorials/add-device
so you can connect to iotroam.
To get Mac address use "network_test.py"
'''


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
EN_R=14 # GPIO14=Pin 19==> Right Connector Pin 2 (looking from the top - back side of charior)
EN_L=11 # GPIO11=Pin 15==> Right Connectot Pin 5 (looking from the top - back side of charior)

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

# setsup encoder sensor
Lsensor = machine.Pin(EN_L, machine.Pin.IN, machine.Pin.PULL_DOWN)
Rsensor = machine.Pin(EN_R, machine.Pin.IN, machine.Pin.PULL_DOWN)

# fitting coefficient for the optimal cubic curve to provide a speed_to_PWM map for each wheel
# to be used in the 'set_wheel_speed(..)' function
coeff_LP = []
coeff_RP = []
coeff_LN = []
coeff_RN = []

# variable to indicate if network connection was succesful
is_connected=False

# loads the local page content
page = open("main.html", "r")
html = page.read()
page.close()

# variables for timing
last_tickL = time.ticks_us()
last_tickR = time.ticks_us()
delta_L=0
delta_R=0
start_time = time.time()
cum_delta_L=0
cum_delta_R=0
num_delta_L=0
num_delta_R=0
# insert a "dead" time where IRQ does not responde after an edge detection to debounce sensor
# assuming max speed 2.5 RPS, with 10 slots ==> max triggers_per_second=2*10*2.5=50
# ==> minimum time between real edges mTBT=20 mS = 200000 us
refractory_time_us=10000 # in microseconds: to debounce the IR triggered interrupt. << mTBT

# variables for wheels speed calibration
I_count=0
Target_I_count=0
last_tick=0
current_tick=0

# more IRQ related variables
trans_HL=0
trans_LH=1
last_LH_trans=0
last_HL_trans=0
pulse_width=0
detected_pulse=False
treshold_H=1000 # in microseconds (10 ms). Shorter impulses are considered noise
threshold_L=1000 


'''
def handle_interrupt(pin): # still in development !!
    global last_tickL, last_tickR,delta_L,delta_R
    global cum_delta_L, cum_delta_R, num_delta_L, num_delta_R
    
    pin_Nr=int((str(pin).split("GPIO")[1])[0:2])

    # updated last_tick
    if(pin_Nr==EN_L):
        result=encoder_filter(Lsensor.value())
        if result: # we got a pulse_width measurement as the sum of high+low long parts
            delta_L=result # to connect to the old code
    if(pin_Nr==EN_R):
        last_tick = last_tickR
        if time.ticks_diff(current_tick, last_tick) < refractory_time_us: # debounce
            return 
        last_tickR=current_tick
        delta_R = time.ticks_diff(current_tick, last_tick)
        cum_delta_R=cum_delta_R+delta_R
        num_delta_R=num_delta_R+1
    
    # Calculate difference
    #delta = time.ticks_diff(current_tick, last_tick)
        
    
    # Determine the type of transition
    # state = "ON" if pin.value() else "OFF"
    
    # Convert microseconds to milliseconds for readability
    #print(f"Transition for pin {pin_Nr} to {state:3} | Interval: {delta / 1000:>8.2f} ms")

    # prints out actual speed measurement
    if (delta_L>0 and delta_R>0):
        print(f"d_L={100000/delta_L:>4.3} d_R={100000/delta_R:>4.3}")
    elif (delta_L>0):
        print(f"d_L={100000/delta_L:>4.3} d_R=n.d.")
    elif (delta_R>0):
        print(f"d_L=n.d. d_R={100000/delta_R:>4.3}")
'''                    
        
def handle_interrupt(pin): # stable, but may not work properly with noisy sensors
    global last_tickL, last_tickR,delta_L,delta_R
    global cum_delta_L, cum_delta_R, num_delta_L, num_delta_R
    
    pin_Nr=int((str(pin).split("GPIO")[1])[0:2])

    # Capture current time in microseconds immediately
    current_tick = time.ticks_us()
    # updated last_tick
    if(pin_Nr==EN_L):
        # does nothing if time difference btw current time and last one is smaller
        # that refractory time. Used to debounce the IR sensor
        last_tick = last_tickL
        if time.ticks_diff(current_tick, last_tick) < refractory_time_us: # debounce
            return 
        last_tickL=current_tick
        delta_L = time.ticks_diff(current_tick, last_tick)
        cum_delta_L=cum_delta_L+delta_L
        num_delta_L=num_delta_L+1
    if(pin_Nr==EN_R):
        last_tick = last_tickR
        if time.ticks_diff(current_tick, last_tick) < refractory_time_us: # debounce
            return 
        last_tickR=current_tick
        delta_R = time.ticks_diff(current_tick, last_tick)
        cum_delta_R=cum_delta_R+delta_R
        num_delta_R=num_delta_R+1
    
    # Calculate difference
    delta = time.ticks_diff(current_tick, last_tick)
        
    
    # Determine the type of transition
    state = "ON" if pin.value() else "OFF"
    
    # Convert microseconds to milliseconds for readability
    #print(f"Transition for pin {pin_Nr} to {state:3} | Interval: {delta / 1000:>8.2f} ms")

    # prints out actual speed measurement
    if (delta_L>0 and delta_R>0):
        print(f"d_L={100000/delta_L:>4.3} d_R={100000/delta_R:>4.3}")
    elif (delta_L>0):
        print(f"d_L={100000/delta_L:>4.3} d_R=n.d.")
    elif (delta_R>0):
        print(f"d_L=n.d. d_R={100000/delta_R:>4.3}")
        
    try:
        print(f"d_L={100000/delta_L:>4.3} d_R={100000/delta_R:>4.3}")
    except:
        pass

def IRQ_counter(pin): # used in the calibration process. May be subject to sensor noise.
    # registers interrupts events in a I_count variable
    # and disables itself when it reaches Target_I_count
    global I_count, Target_I_count, last_tick, current_tick
    if I_count==0:
        last_tick=time.ticks_us() # starts measuring time
        
    
    if I_count==Target_I_count:
        # stops timer when the specific number of transition is reached
        # this is detected when the NEXT transition over Target_I_count is detected
        current_tick=time.ticks_us()
        
    I_count=I_count+1

    #diagnostic
    #print(f"Counted {I_count} transitions")    

# function controlling servos
def MoveForward(power=500,Stime=5):
     global last_tick
    # power is not used here, values should be btw 1000 and 9000 (from full forward to full reverse)
    # 5000 should be motor stopped. To be tested.
    # https://microcontrollerslab.com/servo-motor-raspberry-pi-pico-micropython/
     LeftMotor.duty_u16(5000+power)
     RightMotor.duty_u16(5000-power)
     last_tickL = time.ticks_us() # for the accounting of the interrupt
     last_tickR = time.ticks_us() # for the accounting of the interrupt
     time.sleep(Stime)
     LeftMotor.duty_u16(5000)
     RightMotor.duty_u16(5000)     

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
    
def set_interrupts():
    print(f"Monitoring transitions on GPIO {EN_L} when wheels spinning.")
    # Configure interrupt to trigger on both rising (OFF->ON) and falling (ON->OFF) edges
    Lsensor.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=handle_interrupt)
    Rsensor.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=handle_interrupt)
    # same but only for ascending fronts
    #Lsensor.irq(trigger=machine.Pin.IRQ_RISING , handler=handle_interrupt)
    #Rsensor.irq(trigger=machine.Pin.IRQ_RISING , handler=handle_interrupt)


def measure_motor_speed(PWM,motor,steps=100):
    # rotates the motor until steps a->b->a transitions are identified
    # (it actually counts 2*step+1 a->b and b->a transitions)
    # where a is the initial encoder state (can be LOW or HIGH)
    # counts stops when the subsequent a->b transition is identified
    global I_count, Target_I_count, last_tick, current_tick
    I_count=0
    Target_I_count=2*steps
    
    control=''
    sensor=''
    
    if str(motor).lower() in ['l','left']:
        control=LeftMotor
        sensor=Lsensor
        # disables the other motor interrupt and redirects the current one
        Rsensor.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=None)
        Lsensor.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=IRQ_counter)        
    if str(motor).lower() in ['r','right']:
        control=RightMotor
        sensor=Rsensor
        # disables the other motor interrupt and redirects the current one
        Lsensor.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=None)
        Rsensor.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=IRQ_counter)        
    if not control:
        return(-1)
    
    # runs the motor and waits until count does not reach 
    control.duty_u16(PWM)
    
    while(I_count<=Target_I_count):
        #print(f"Counted {I_count} transitions out of {Target_I_count}")
        time.sleep_ms(10)
    time.sleep_ms(500)
    
    # stps and reports results
    sensor.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=None)
    control.duty_u16(5000)
    
    delta = time.ticks_diff(current_tick, last_tick)

    print(f"Counted {Target_I_count/2/10} rotations over {delta / 1000:>8.2f} milliseconds")
    # restores default IRQ handlers at the end
    #Lsensor.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=handle_interrupt)
    #Rsensor.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=handle_interrupt)

    return(Target_I_count/2/10,delta/1000)
    # /2 as there are two impulses for cycle, /10 as there are 10 slots in the wheel
        
def calibrate(PWM_START, PWM_END, PWM_STEP, side='L', steps=100):
    # measures over 'step' impulses, the rotation period of the 'side'=['L' or 'R'] motor
    # and returns an array containing Rotation Per Second (RPS) and the tested PWM values 
    # (calculated over a number of rotations defined by 'steps'/10)
    # timing results have been verified with external clock and are correct
    tests=range(PWM_START, PWM_END, PWM_STEP)
    results=[]
    for each in tests:
        rotations,time_ms=measure_motor_speed(each,side,steps)
        results.append((rotations/(time_ms/1000),(each-5000)/2000)) #(RPS, (PWM-5000)/2000)

    return(results)

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
    
def set_motor_speed(RPS_L,RPS_R,t_sleep=5):
    PWM_L,PWM_R=calculate_PWM(RPS_L,RPS_R)
    global cum_delta_L, cum_delta_R, num_delta_L, num_delta_R
    global last_tickL, last_tickR,delta_L,delta_R

    cum_delta_L=0
    cum_delta_R=0
    num_delta_L=0
    num_delta_R=0
    last_tickL=time.ticks_us()
    last_tickR=time.ticks_us()
    delta_L=0
    delta_R=0
    
    # handle separately zero speed case
    if RPS_L==0:
        PWM_L=5000
    if RPS_R==0:
        PWM_R=5000
    
    Lsensor.irq(trigger=machine.Pin.IRQ_RISING , handler=handle_interrupt)
    Rsensor.irq(trigger=machine.Pin.IRQ_RISING , handler=handle_interrupt)

    LeftMotor.duty_u16(PWM_L)
    RightMotor.duty_u16(PWM_R)
    time.sleep(t_sleep)
    Lsensor.irq(trigger=machine.Pin.IRQ_RISING , handler=None)
    Rsensor.irq(trigger=machine.Pin.IRQ_RISING , handler=None)

    LeftMotor.duty_u16(5000)
    RightMotor.duty_u16(5000)
    try:
        #  d_L=cum_delta_L/num_delta_L
        RPS_L=100000*num_delta_L/cum_delta_L
    except:
        RPS_L=0
    try:
        #d_R=cum_delta_R/max(num_delta_R,1)
        RPS_R=100000*num_delta_R/cum_delta_R
    except:
        RPS_R=0

    print(f"avg RPS_L={RPS_L:>4.3} avg RPS_R={RPS_R:>4.3}")


def encoder_filter(trans_type):
    # checking the last two opposite fronts (H->L and l->H) tries to detect if a long
    # HIGH or LOW level is detected (exceeding the relevant threshold)
    # if not, it just updtes the last_XX_transition time
    # if yes, it reconstruct the full pulse by adding the duration of the long HIGH part
    # with the duration of the long LOW part. short pulses are ignored (for now)
    global last_HL_trans , last_LH_trans , pulse_width, detected_pulse
    # Capture current time in microseconds immediately
    current_tick = time.ticks_us()
    # transition time, can be trans_HL or trans_LH
    if trans_type==trans_HL:
        width_H=current_tick-last_LH_trans
        last_HL_trans=current_tick # updates global variable
        if width_H>treshold_H: # we got a long HIGH period that just finished
            pulse_width=width_H # the pulse length measure starts with the HIGH part
            detected_pulse=False # we are starting now pulse detection
    if trans_type==trans_LH:
        width_L=current_tick-last_HL_trans
        last_LH_trans=current_tick
        if width_L>threshold_L: # we got a long LOW period that just finished
            print(f"LOW_p={pulse_width} HIGH_p={width_L}")
            pulse_width=pulse_width+width_L # pulse detection finishes with the LOW part
            detected_pulse=True
    if detected_pulse:
        return(pulse_width)
    else:
        return(None)
   
    
def autocalibrate():
    # performs multi-PWM calibration for both wheels and, using the results,
    # calculates separately the best speed_to_PWM fitting function to be then used
    # to set the wheel speed (in RPS)
    global coeff_LP, coeff_RP, coeff_LN, coeff_RN
    # measures the speed_to_PWM relation
    # note that the first number is pure RPS (Rotations Per Second) while the second
    # one is a scaled PWM in the form of (PWM-5000)/2000
    results_LP=calibrate(5200,6601,200, 'Left')
    results_RP=calibrate(4600,3100,-200,'Right')
    # results_RP=[(0.3260246, -0.2), (0.6752449, -0.3), (0.9138268, -0.4), (1.132149, -0.5), (1.287252, -0.6), (1.4521, -0.7), (1.605211, -0.8), (1.779282, -0.9)]
    # reverse speed calibration
    results_LN=calibrate(4600,3100,-200, 'Left')
    results_RN=calibrate(5200,6601,200,'Right')
    # results_LN=[(0.1868922, -0.2), (0.4940495, -0.3), (0.7126633, -0.4), (0.8818153, -0.5), (1.026694, -0.6), (1.19801, -0.7), (1.413726, -0.8), (1.586388, -0.9)]
    # results_RN=[(0.4401469, 0.1), (0.76094, 0.2), (0.9941456, 0.3), (1.189848, 0.4), (1.360654, 0.5), (1.516376, 0.6), (1.685751, 0.7), (1.764554, 0.8)]

    # fits the best cubic function through each set of points    
    coeff_LP=polyfit3(results_LP)
    # coeff_LP=(-0.1777802, 0.6523898, -0.1553434, 0.08150997)
    coeff_RP=polyfit3(results_RP)
    # coeff_RP=(0.0814823, -0.4187451, 0.08430676, -0.1871835)
    coeff_LN=polyfit3(results_LN)
    # coeff_LN=(0.2038952, -0.6254874, 0.03229254, -0.1854067)
    coeff_RN=polyfit3(results_RN)
    # coeff_RN=(0.04240555, 0.0646009, 0.2027688, -0.006709341)
    
    # saves new calibration coefficients
    with open("wheels_calibration.py","w") as f:
        f.write("coeff_LP="+str(coeff_LP)+'\n')
        f.write("coeff_RP="+str(coeff_RP)+'\n')
        f.write("coeff_LN="+str(coeff_LN)+'\n')
        f.write("coeff_RN="+str(coeff_RN)+'\n')
        f.write("default_values=False")        
    
#     # calculates an example
#     RPS=1
#     PWM_L,PWM_R=calculate_PWM(RPS,RPS)
#     print(f"Example: to set speed at {RPS} round per second use PWM_L={PWM_L} and PWM_R={PWM_R}")


def load_calibration():
    # uses coefficients stored in wheel_calibration.py
    # overwrites those with new calibration.
    # fallsback to default if no calibration file is present.
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
                    #print(f"setting speed_L={speed_L} speed_R={speed_R}")
                    set_motor_speed(speed_L,speed_R,time_float)
                    #MoveForward(25,5)
                    found=True
                if str(line).find("?PRESS_5=CALIBRATE") !=-1:
                    print("Command CALIBRATE received")
                    autocalibrate()
                    found=True       
        # we process the response file, We can add placeholders to turn change the page aspect
        response=html # default page, placeholders needs to be replaced before submitting
        # send the page
        cl.send('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
        cl.send(response)
        cl.close()
    else:
        print("No connection is present. No web pagina is being server")


## ENTRY POINT FOR MAIN CODE

# load calibration file for motors if present, fallback to a a default if not present
load_calibration()
# try connecting to network and, if ok, start listening to port 80
# global variable is_connected contains network connection status
connect_to_network()
# tries to serve the web page (and responds to API calls) for control
# process is blocking
serve_pagina()
