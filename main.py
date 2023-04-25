from machine import Pin, PWM, Timer
import utime

def clamp(val, min, max):
    if val < min:
        val = min
    if val > max:
        val = max
    return val

class WirelessComms:
    def __init__(self):
        pass
    def connect(user, net_pass):
        pass

class QuadratureEncoder:
    def __init__(self, pin_a, pin_b, gear_ratio, cpr, polarity):
        self.pin_a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self.pin_b = Pin(pin_b, Pin.IN, Pin.PULL_UP)
        self.gear_ratio = gear_ratio
        self.cpr = cpr
        self.pol = polarity

        self.state_a = self.pin_a.value()
        self.state_b = self.pin_b.value()
        
        self.count = 0
        self.position = 0 # in revolutions
        
        self.pin_a.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=self.callback)
        self.pin_b.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=self.callback)

    def callback(self, pin):
        
        
        new_state_a = self.pin_a.value()
        new_state_b = self.pin_b.value()
        
        #prevent erroneous callbacks
        if (new_state_a == self.state_a and new_state_b == self.state_b):
            return
        
        #print(self.state_a, self.state_b, "-->", new_state_a, new_state_b)
        
        #TODO incorporate polarity
        # Quadrature Encoder State Machine
        if self.state_a == 0 and self.state_b == 0:
            if new_state_a == 1 and new_state_b == 0:
                self.count += 1 * self.pol
            elif new_state_a == 0 and new_state_b == 1:
                self.count -= 1 * self.pol
            else:
                print("ENCODER ERROR 1!")
        elif self.state_a == 1 and self.state_b == 0:
            if new_state_a == 1 and new_state_b == 1:
                self.count += 1 * self.pol
            elif new_state_a == 0 and new_state_b == 0:
                self.count -= 1 * self.pol
            else:
                print("ENCODER ERROR 2!")
        elif self.state_a == 1 and self.state_b == 1:
            if new_state_a == 0 and new_state_b == 1:
                self.count += 1 * self.pol
            elif new_state_a == 1 and new_state_b == 0:
                self.count -= 1 * self.pol
            else:
                print("ENCODER ERROR 3!")
        else:
            if new_state_a == 0 and new_state_b == 0:
                self.count += 1 * self.pol
            elif new_state_a == 1 and new_state_b == 1:
                self.count -= 1 * self.pol
            else:
                print("ENCODER ERROR 4!")
        
        #set polarity
        #self.count *= self.polarity
        
        self.state_a = new_state_a
        self.state_b = new_state_b
        
        self.position = self.count / (self.cpr * self.gear_ratio)
        #print(self.position)

class Motor:
    def __init__(self, in1_pin, in2_pin, pwm_pin, stby_pin, encoder = None, polarity = 1):
        self.in1_pin = Pin(in1_pin, Pin.OUT)
        self.in2_pin = Pin(in2_pin, Pin.OUT)
        
        #init pwm
        pwm_pin = Pin(pwm_pin, Pin.OUT)
        self.pwm = PWM(pwm_pin)
        self.pwm.freq(1000)
        self.pwm.duty_u16(0)
        
        #enable motor driver
        self.stby_pin = Pin(stby_pin, Pin.OUT)
        self.stby_pin.value(1)
        
        self.encoder = encoder
        self.pol = polarity
            
        #NOT CURRENTLY USING PID
        #self.pid = PID(3.0, 1.0, 0)
        
        self.max_duty = 90
        
        self.prev_diff = 0
    
    def drive_to_setpoint(self, setpoint, thresh=0.1, duty_magnitude=80):
        diff = self.encoder.position - setpoint
        
        if diff * self.prev_diff < 0 or diff == 0: #abs(diff) <= thresh:
            self.brake()
            self.prev_diff = 0
            return True
        else:
            if self.encoder.position < setpoint:
                #forward
                self.drive(duty_magnitude)
            else:
                #reverse
                self.drive(-duty_magnitude)
                
        self.prev_diff = diff
        return False
            
        
    
    def drive(self, duty):
        duty = duty * self.pol
        if duty > 0:
            self.forward(duty)
        elif duty < 0:
            self.reverse(-duty)
        else:
            self.brake()
        
    def forward(self, duty):
        duty = clamp(duty, 0, self.max_duty)
        self.in1_pin.value(1)
        self.in2_pin.value(0)
        self.pwm.duty_u16(int((duty/100)*65535))
        
    def reverse(self, duty):
        duty = clamp(duty, 0, self.max_duty)
        self.in1_pin.value(0)
        self.in2_pin.value(1)
        self.pwm.duty_u16(int((duty/100)*65535))
    
    def brake(self):
        self.in1_pin.value(1)
        self.in2_pin.value(1)
        self.pwm.duty_u16(0)
        
        
        
enc1 = QuadratureEncoder(4, 5, gear_ratio=75.0, cpr=12.0, polarity=1)
motor1 = Motor(1, 2, 3, 0, enc1, polarity = -1)

enc2 = QuadratureEncoder(9, 10, gear_ratio=75.0, cpr=12.0, polarity=1)
motor2 = Motor(6, 7, 8, 0, enc2)

enc3 = QuadratureEncoder(14, 15, gear_ratio=75.0, cpr=12.0, polarity=-1)
motor3 = Motor(11, 12, 13, 0, enc3)

cMotor1 = Motor(16, 17, 18, 22)
cMotor2 = Motor(19, 20, 21, 22)


"""
motor1.encoder.position = 0.0
motor2.encoder.position = 0.0
motor3.encoder.position = 0.0
#TODO - tune effective length, better polarity
setpoint = 0
while True:
    i = input()
    if i == 'q':
        motor1.brake()
        motor2.brake()
        motor3.brake()
        cMotor1.brake()
        cMotor2.brake()
        motor1.encoder.position = 0.0
        motor2.encoder.position = 0.0
        motor3.encoder.position = 0.0
    elif i == 'w':
        setpoint += 1
        while True:
            ret1 = motor1.drive_to_setpoint(-setpoint)
            ret2 = motor2.drive_to_setpoint(setpoint, thresh=0.05)
            ret3 = motor3.drive_to_setpoint(setpoint, thresh=0.05)
            
            print(motor1.encoder.position, motor2.encoder.position, motor3.encoder.position)
    
            if ret1 and ret2 and ret3:
                break
    elif i == 's':
        setpoint -= 1
        while True:
            ret1 = motor1.drive_to_setpoint(-setpoint, duty_magnitude=20)
            ret2 = motor2.drive_to_setpoint(setpoint, duty_magnitude=20)
            ret3 = motor3.drive_to_setpoint(setpoint, duty_magnitude=20)
            
            print(motor1.encoder.position, motor2.encoder.position, motor3.encoder.position)
    
            if ret1 and ret2 and ret3:
                break
    elif i == 'r':
        motor1.encoder.position = 0.0
        motor2.encoder.position = 0.0
        motor3.encoder.position = 0.0
    elif i == '1f':
        motor1.drive(-30)
    elif i == '1r':
        motor1.drive(30)
    elif i == '2f':
        motor2.drive(30)
    elif i == '2r':
        motor2.drive(-30)
    elif i == '3f':
        motor3.drive(30)
    elif i == '3r':
        motor3.drive(-30)
    elif i == 'nf':
        cMotor1.drive(50)
    elif i == 'nr':
        cMotor1.drive(-50)
    elif i =='rf':
        cMotor2.drive(50)
    elif i == 'rr':
        cMotor2.drive(-50)
    else:
        motor1.brake()
        motor2.brake()
        motor3.brake()
        cMotor1.brake()
        cMotor2.brake()
"""


"""
WIGGLES STATE MACHINE
States

0 - close nose
1 - open nose
2 - close rear
3 - open rear
4 - contract body
5 - open body
"""

motor1.brake()
motor2.brake()
motor3.brake()
cMotor1.brake()
cMotor2.brake()
motor1.encoder.position = 0.0
motor2.encoder.position = 0.0
motor3.encoder.position = 0.0

while True:
    i = input()
    if i == 'w':
        break
    

cycle = [0, 3, 4, 2, 1, 5]
i = 0
close_setpoint = 1.4
open_setpoint = 0
close_time = 1.8 * 1000
open_time = 1 * 1000

cycle_count = 0

div = 16.0
move_unit = (close_setpoint-open_setpoint)/div

while True:     
    state = cycle[i]
    print("STATE = " , state)
    if state == 0:
        # close nose
        t = utime.ticks_ms()
        while utime.ticks_ms() < t + close_time:
            #print(str(utime.ticks_ms()), str(t+close_time))
            cMotor1.drive(50)
        cMotor1.brake()
        
    elif state == 1:
        # open nose
        t = utime.ticks_ms()
        while utime.ticks_ms() < t + open_time:
            cMotor1.drive(-50)
        cMotor1.brake()
        
    elif state == 2:
        # close rear
        t = utime.ticks_ms()
        while utime.ticks_ms() < t + close_time:
            cMotor2.drive(50)
        cMotor2.brake()
        
    elif state == 3:
        # open rear
        t = utime.ticks_ms()
        while utime.ticks_ms() < t + open_time:
            cMotor2.drive(-50)
        cMotor2.brake()
        
    elif state == 4:
        #close body
        """
        while True:
            ret1 = motor1.drive_to_setpoint(-close_setpoint)
            ret2 = motor2.drive_to_setpoint(close_setpoint)
            ret3 = motor3.drive_to_setpoint(close_setpoint)
            if ret1 and ret2 and ret3:
                break
        """
        
        
        for j in range(div):
            t = utime.ticks_ms()    
                                           
            a = motor1.encoder.position+move_unit
            b = motor2.encoder.position+move_unit
            c = motor3.encoder.position+move_unit
            
            print("Position goals")
            print(a)
            print(b)
            print(c)
            
            if (motor1.encoder.position < close_setpoint):
                while True and utime.ticks_diff(utime.ticks_ms(), t) < 2000:                                
                    ret1 = motor1.drive_to_setpoint(a)
                    if ret1:
                        
                        break
            t = utime.ticks_ms()            
            if (motor2.encoder.position < close_setpoint):
                while True and utime.ticks_diff(utime.ticks_ms(), t) < 2000:
                    
                    ret2 = motor2.drive_to_setpoint(b)
                    if ret2:
                        break
            t = utime.ticks_ms()    
            if (motor3.encoder.position < close_setpoint):
                while True and utime.ticks_diff(utime.ticks_ms(), t) < 2000:
                    ret3 = motor3.drive_to_setpoint(c)
                    if ret3:
                        break
            #utime.sleep_ms(1000)
        
            
    elif state == 5:
        #open_body
        """
        while True:
            ret1 = motor1.drive_to_setpoint(-open_setpoint, duty_magnitude=20)
            ret2 = motor2.drive_to_setpoint(open_setpoint, duty_magnitude=20)
            ret3 = motor3.drive_to_setpoint(open_setpoint, duty_magnitude=20)
            if ret1 and ret2 and ret3:
                break
        """
        
        for j in range(div):
            t = utime.ticks_ms()    
            a = motor1.encoder.position-move_unit
            b = motor2.encoder.position-move_unit
            c = motor3.encoder.position-move_unit
            print("Position goals")
            print(a)
            print(b)
            print(c)
            
            if (motor1.encoder.position > open_setpoint):
                while True and utime.ticks_diff(utime.ticks_ms(), t) < 2000:
                    ret1 = motor1.drive_to_setpoint(a)
                    if ret1:
                        break
            t = utime.ticks_ms()    
            if (motor2.encoder.position > open_setpoint):
                while True and utime.ticks_diff(utime.ticks_ms(), t) < 2000:
                    ret2 = motor2.drive_to_setpoint(b)
                    if ret2:
                        break
            t = utime.ticks_ms()                        
            if (motor3.encoder.position > open_setpoint):
                while True and utime.ticks_diff(utime.ticks_ms(), t) < 2000:
                    ret3 = motor3.drive_to_setpoint(c)
                    if ret3:
                        break
            
            #utime.sleep_ms(1000)
                
    else:
        motor1.brake()
        motor2.brake()
        motor3.brake()
        cMotor1.brake()
        cMotor2.brake()
    
    i += 1
    
    if i == 1 and cycle_count == 0:
        #skip rear open on first cycle
        i += 1
    
    if i > 5:
        print("RESETTING")
        cycle_count += 1
        i = 0



        
        