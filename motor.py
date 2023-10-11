from machine import Pin, PWM, Timer
import utime
from util import clamp

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
        
        
        self.state_a = new_state_a
        self.state_b = new_state_b
        
        self.position = self.count / (self.cpr * self.gear_ratio)

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
        
        