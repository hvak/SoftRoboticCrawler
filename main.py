from machine import Pin, PWM, Timer

def clamp(val, min, max):
    if val < min:
        val = min
    if val > max:
        val = max
    return val

class QuadratureEncoder:
    def __init__(self, pin_a, pin_b, gear_ratio):
        self.pin_a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self.pin_b = Pin(pin_b, Pin.IN, Pin.PULL_UP)
        self.gear_ratio = gear_ratio
        self.position = 0
        self.cpr = 12.0

        self.state_a = 0
        self.state_b = 0
        self.count = 0
        
        self.pin_a.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=self.callback)
        self.pin_b.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=self.callback)

    def callback(self, pin):
        new_state_a = pin.value()
        new_state_b = pin.value()
        
        # Quadrature Encoder State Machine
        if self.state_a == 0 and self.state_b == 0:
            if new_state_a == 1 and new_state_b == 0:
                self.count += 1
            elif new_state_a == 0 and new_state_b == 1:
                self.count -= 1
            else:
                print("ENCODER ERROR!")
        elif self.state_a == 1 and self.state_b == 0:
            if new_state_a == 1 and new_state_b == 1:
                self.count += 1
            elif new_state_a == 0 and new_state_b == 0:
                self.count -= 1
            else:
                print("ENCODER ERROR!")
        elif self.state_a == 1 and self.state_b == 1:
            if new_state_a == 0 and new_state_b == 1:
                self.count += 1
            elif new_state_a == 1 and new_state_b == 0:
                self.count -= 1
            else:
                print("ENCODER ERROR!")
        else:
            if new_state_a == 0 and new_state_b == 0:
                self.count += 1
            elif new_state_a == 1 and new_state_b == 1:
                self.count -= 1
            else:
                print("ENCODER ERROR!")
        
        self.state_a = new_state_a
        self.state_b = new_state_b
        
        self.position = self.count / (self.cpr * self.gear_ratio)

class Motor:
    def __init__(self, in1_pin, in2_pin, pwm_pin, stby_pin):
        self.in1_pin = Pin(in1_pin, Pin.OUT)
        self.in2_pin = Pin(in2_pin, Pin.OUT)
        pwm_pin = Pin(pwm_pin, Pin.OUT)
        self.pwm = PWM(pwm_pin)
        self.pwm.freq(1000)
        
        #enable motor driver
        self.stby_pin = Pin(stby_pin, Pin.OUT)
        self.stby_pin.value(1)
        
        self.max_duty = 90
    
    def drive(self, duty):
        if duty > 0:
            self.forward(duty)
        elif duty < 0:
            self.reverse(duty)
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
        self.pwm.duty_u16((duty/100)*65_535)
    
    def brake(self):
        self.in1_pin.value(1)
        self.in2_pin.value(1)
        self.pwm.duty_u16(0)
        
        
        
motor1 = Motor(1, 2, 3, 0)
enc = QuadratureEncoder(4, 5, 75.0)
motor.drive(50)
