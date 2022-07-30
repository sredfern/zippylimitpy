import time
from machine import Pin, PWM
import rp2
import _thread
import utime
import os
import json

def globalCounter():
    global counter
    counter += 1

class seven:
    def __init__(self):
        self.led1 = Pin(15, Pin.OUT, Pin.OPEN_DRAIN) #dp
        self.led2 = Pin(14, Pin.OUT, Pin.OPEN_DRAIN) #c
        self.led3 = Pin(10, Pin.OUT, Pin.OPEN_DRAIN) #b
        self.led4 = Pin(11, Pin.OUT, Pin.OPEN_DRAIN) #a
        self.led5 = Pin(21, Pin.OUT, Pin.OPEN_DRAIN) #g
        self.led6 = Pin(18, Pin.OUT, Pin.OPEN_DRAIN) #f
        self.led7 = Pin(17, Pin.OUT, Pin.OPEN_DRAIN) #d
        self.led8 = Pin(16, Pin.OUT, Pin.OPEN_DRAIN) #e
    def off(self):
        self.led1.on() #dp
        self.led2.on() #c
        self.led3.on() #b
        self.led4.on() #a
        self.led5.on() #g
        self.led6.on() #f
        self.led7.on() #d
        self.led8.on() #e
    def number(self, num):
        if num == 0: 
            self.led1.off() #dp
            self.led2.on() #c
            self.led3.on() #b
            self.led4.on() #a
            self.led5.on() #g
            self.led6.on() #f
            self.led7.on() #d
            self.led8.on() #e
        if num == 1: 
            self.led1.on() #dp
            self.led2.off() #c
            self.led3.off() #b
            self.led4.on() #a
            self.led5.on() #g
            self.led6.on() #f
            self.led7.on() #d
            self.led8.on() #e
        if num == 2:
            self.led1.on() #dp
            self.led2.on() #c
            self.led3.off() #b
            self.led4.off() #a
            self.led5.off() #g
            self.led6.on() #f
            self.led7.off() #d
            self.led8.off() #e
        if num == 3:
            self.led1.on() #dp
            self.led2.off() #c
            self.led3.off() #b
            self.led4.off() #a
            self.led5.off() #g
            self.led6.on() #f
            self.led7.off() #d
            self.led8.on() #e
        if num == 4:
            self.led1.on() #dp
            self.led2.off() #c
            self.led3.off() #b
            self.led4.on() #a
            self.led5.off() #g
            self.led6.off() #f
            self.led7.on() #d
            self.led8.on() #e
        if num == 5:
            self.led1.on() #dp
            self.led2.off() #c
            self.led3.on() #b
            self.led4.off() #a
            self.led5.off() #g
            self.led6.off() #f
            self.led7.off() #d
            self.led8.on() #e
        if num == 6:
            self.led1.on() #dp
            self.led2.off() #c
            self.led3.on() #b
            self.led4.off() #a
            self.led5.off() #g
            self.led6.off() #f
            self.led7.off() #d
            self.led8.off() #e
        if num == 7:
            self.led1.on() #dp
            self.led2.off() #c
            self.led3.off() #b
            self.led4.off() #a
            self.led5.on() #g
            self.led6.on() #f
            self.led7.on() #d
            self.led8.on() #e
        if num == 8:
            self.led1.on() #dp
            self.led2.off() #c
            self.led3.off() #b
            self.led4.off() #a
            self.led5.off() #g
            self.led6.off() #f
            self.led7.off() #d
            self.led8.off() #e
        if num == 9:
            self.led1.on() #dp
            self.led2.off() #c
            self.led3.off() #b
            self.led4.off() #a
            self.led5.off() #g
            self.led6.off() #f
            self.led7.off() #d
            self.led8.on() #e
        if num == 10:
            self.led1.on() #dp
            self.led2.off() #c
            self.led3.off() #b
            self.led4.off() #a
            self.led5.on() #g
            self.led6.off() #f
            self.led7.off() #d
            self.led8.off() #e
    def letter(self, let):
        if let == 'A':
            self.led1.on() #dp
            self.led2.off() #c
            self.led3.off() #b
            self.led4.off() #a
            self.led5.off() #g
            self.led6.off() #f
            self.led7.on() #d
            self.led8.off() #e
        if let == 'c':
            self.led1.on() #dp
            self.led2.on() #c
            self.led3.on() #b
            self.led4.on() #a
            self.led5.off() #g
            self.led6.on() #f
            self.led7.off() #d
            self.led8.off() #e
        if let == 'w':
            self.led1.off() #dp
            self.led2.on() #c
            self.led3.on() #b
            self.led4.on() #a
            self.led5.on() #g
            self.led6.on() #f
            self.led7.on() #d
            self.led8.on() #e
    def sign(self, sig):
        if sig == 'up':
            self.led1.on() #dp
            self.led2.on() #c
            self.led3.on() #b
            self.led4.off() #a
            self.led5.on() #g
            self.led6.on() #f
            self.led7.on() #d
            self.led8.on() #e
        if sig == 'mid':
            self.led1.on() #dp
            self.led2.on() #c
            self.led3.on() #b
            self.led4.on() #a
            self.led5.off() #g
            self.led6.on() #f
            self.led7.on() #d
            self.led8.on() #e
        if sig == 'low':
            self.led1.on() #dp
            self.led2.on() #c
            self.led3.on() #b
            self.led4.on() #a
            self.led5.on() #g
            self.led6.on() #f
            self.led7.off() #d
            self.led8.on() #e
        if sig == 'error':
            self.led1.off() #dp
            self.led2.on() #c
            self.led3.on() #b
            self.led4.off() #a
            self.led5.on() #g
            self.led6.on() #f
            self.led7.off() #d
            self.led8.on() #e
        if sig == 'bus1':
            self.led1.on() #dp
            self.led2.off() #c
            self.led3.off() #b
            self.led4.on() #a
            self.led5.off() #g
            self.led6.on() #f
            self.led7.on() #d
            self.led8.on() #e
        if sig == 'bus2':
            self.led1.on() #dp
            self.led2.on() #c
            self.led3.on() #b
            self.led4.on() #a
            self.led5.off() #g
            self.led6.off() #f
            self.led7.on() #d
            self.led8.off() #e

class Servo:
    """ A simple class for controlling a 9g servo with the Raspberry Pi Pico.
    Attributes:
        minVal: An integer denoting the minimum duty value for the servo motor.
        maxVal: An integer denoting the maximum duty value for the servo motor.
    """

    def __init__(self, pin: int or Pin or PWM, minVal=2500, maxVal=7500):
        """ Creates a new Servo Object.
        args:
            pin (int or machine.Pin or machine.PWM): Either an integer denoting the number of the GPIO pin or an already constructed Pin or PWM object that is connected to the servo.
            minVal (int): Optional, denotes the minimum duty value to be used for this servo.
            maxVal (int): Optional, denotes the maximum duty value to be used for this servo.
        """

        if isinstance(pin, int):
            pin = Pin(pin, Pin.OUT)
        if isinstance(pin, Pin):
            self.__pwm = PWM(pin)
        if isinstance(pin, PWM):
            self.__pwm = pin
        self.__pwm.freq(50)
        self.minVal = minVal
        self.maxVal = maxVal

    def deinit(self):
        """ Deinitializes the underlying PWM object.
        """
        self.__pwm.deinit()

    def goto(self, value: int):
        """ Moves the servo to the specified position.
        args:
            value (int): The position to move to, represented by a value from 0 to 1024 (inclusive).
        """
        if value < 0:
            value = 0
        if value > 1024:
            value = 1024
        delta = self.maxVal-self.minVal
        target = int(self.minVal + ((value / 1024) * delta))
        self.__pwm.duty_u16(target)

    def middle(self):
        """ Moves the servo to the middle.
        """
        self.goto(512)

    def free(self):
        """ Allows the servo to be moved freely.
        """
        self.__pwm.duty_u16(0)

#
# Filesystem tools 
#
def saveCalibration():
    print(os.listdir())
    if 'cal.json' in os.listdir():
        os.remove('cal.json')
    global max 
    global min
    global mid
    inputObj = {"min": min,
                "max": max,
                "mid": mid}
    print(json.dumps(inputObj))
    f = open('cal.json', 'w')
    f.write(json.dumps(inputObj))
    f.close()
    global start_write
    start_write = False
    dig.off()

def readCalibration():
    if 'cal.json' not in os.listdir():
        dig.sign("error")
        time.sleep(2)
        calibrate()
        return
    f = open('cal.json')
    values = json.loads(f.read())
    print(values)
    f.close()
    global max
    max = values['max']
    global min
    min = values['min']
    global mid
    mid = values['mid']
#
# Calibrate function
#
def calibrate():
    global start_calibrate
    calibrate_stage = 0
    start_calibrate = True
    while start_calibrate == True:
        if pin_button.value() == 0 and calibrate_stage == 0:
            calibrate_stage = 1
        baton.acquire()
        r = result
        baton.release()
        if calibrate_stage in (1,2):
            dig.sign("up")
            if pin_button.value() == 1 and calibrate_stage == 1:
                calibrate_stage = 2
            if pin_button.value() == 0 and calibrate_stage == 2:
                global max
                max = avg_calculator()
                print(avg_calculator())
                calibrate_stage = 3
        if calibrate_stage in (3,4):
            dig.sign("low")
            if pin_button.value() == 1 and calibrate_stage == 3:
                calibrate_stage = 4
            if pin_button.value() == 0 and calibrate_stage == 4:
                global min 
                min = avg_calculator()
                print(avg_calculator())
                calibrate_stage = 5
        if calibrate_stage in (5,6):
            dig.sign("mid")
            if pin_button.value() == 1 and calibrate_stage == 5:
                calibrate_stage = 6
            if pin_button.value() == 0 and calibrate_stage == 6:
                global mid
                mid = avg_calculator()
                print(avg_calculator())
                print("min: ",min," max: ",max," mid: ",mid)
                start_calibrate = False
                dig.off()
        # print("stage: ",str(main_stage),", calibration state:",str(calibrate_stage),"Button state: ",pin_button.value())
        time.sleep(0.1)

#
# Avg Calculator is used to ensure no extreme values are selected during calibration
# 
def avg_calculator():
    mov_avg_array = []
    max_array_size = 10
    flip = True
    while len(mov_avg_array) < max_array_size:
        baton.acquire()
        r = result
        baton.release()
        mov_avg_array.append(r)
        if flip == True:
            dig.sign('bus1')
            flip = False
        else:
            dig.sign('bus2')
            flip = True
        time.sleep(0.03)
    calcValue = round(sum(mov_avg_array)/(max_array_size+1),0)
    print("calibrated value: ",calcValue)
    return int(calcValue)

#
# Servo Value Reader
#
@rp2.asm_pio()
def pulsewidth():
    wrap_target()
    wait(1, pin, 0)                       # 0
    set(x, 0)                             # 1
    jmp(x_dec, "3")                       # 2
    label("3")
    jmp(x_dec, "4")                       # 3
    label("4")
    jmp(pin, "3")                         # 4
    mov(isr, x)                           # 5
    push(isr, block)                         # 6
    irq( 0)                         # 7
    wrap()

result = 0

#
# PIO Servo read
baton = _thread.allocate_lock()
def handler(sm):
    global result
    # x-reg counts down
    value = 0x100000000 - sm.get()
    baton.acquire()               
    result = value  # 0.5 us resolution, so expect 2000 to 4000
                    #                       for    1ms  to 2ms
    baton.release()
    
#
# Servo Read
pin3 = Pin(3, Pin.IN, Pin.PULL_UP)
sm0 = rp2.StateMachine(0, pulsewidth, freq=4_000_000, in_base=pin3, jmp_pin=pin3)
sm0.irq(handler)
sm0.active(1)

# min value: 2125 max: 3881
s1 = Servo(2)

pin_pot = machine.ADC(27) # max value 65535, min value 272

# 
pin_button = Pin(7, Pin.IN, Pin.PULL_DOWN)
#
# Throttle control variables
throttle_out = 90

# Menu logic latches
button_latch = False
start_calibrate = False
start_write = False

# Timer for understanding how long button is pressed
start_press_time = utime.ticks_ms()

#Starter variables
min = 0
max = 0
mid = 0

#
# Digit object
dig = seven()
dig.off()

while True:
    baton.acquire()
    r = result
    baton.release()
    while min == 0 or max == 0 or mid == 0:
        readCalibration()
    # catch for Calibrate or write functions
    if start_calibrate == True and pin_button.value() == 0:
        calibrate()
    if start_write == True and pin_button.value() == 0:
        saveCalibration()
    # button press catch
    if pin_button.value() == 1:
        # First state, re-set middle
        if pin_button.value() == 1 and button_latch == False:
            button_latch = True
            start_press_time = utime.ticks_ms()
        latch_time = utime.ticks_ms() - start_press_time
        if  latch_time < 400:
            dig.off()
            start_calibrate = False
            start_write = False
        if latch_time > 400 and latch_time < 2000:
            dig.letter('c')
            start_calibrate = True
            start_write = False
        if latch_time > 2000:
            dig.letter('w')
            start_calibrate = False
            start_write = True
        # print(utime.ticks_ms() - start_press_time) # useful if you want to see the timing.
    # button latch clear
    if pin_button.value() == 0 and button_latch == True:
        button_latch = False
        dig.off()
    #
    # throttle logic here
    throttle_percent = (r-2000)/2000 # lowering read from 2000 base to 0
    pot_value = pin_pot.read_u16() # read pot value
    per_decrease = round(1.0*pot_value/65535,2) # Turn to percentage
    throttle_out = ((r-mid)*per_decrease)+mid # Remove mid read from throttle read. Apply percent change then re-add mid.
    throttle_scaled = (throttle_out-min)/(max-min) #Re-scale throttle value to within the min-max envolope. 
    if pin_button.value() == 0 and button_latch == False: # Display current decrease value
        dig.number(int(per_decrease*10))
    s1.goto(int(throttle_scaled)*1024) # Push scaled servo value on 1024 base. 
    # print("raw: ",r,"decrease:",per_decrease," output: ",throttle_scaled, "min, max, mid", min, max, mid)
    time.sleep(0.001)