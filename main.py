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

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        self.proportional = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.prev_error = 0
        
        self.prev_time = 0
        
        
    
    def update(self, current_pos, desired_pos):
        
        t = utime.ticks_ms()
        if (t == self.prev_time):
            return       
        dt = utime.ticks_diff(t, self.prev_time) / 1000.0
        self.prev_time = t
        
        error = desired_pos - current_pos
        
        print(current_pos, desired_pos, error)
        
        self.proportional = self.Kp * error
        self.integral += float(self. Ki * error * dt)
        self.derivative = float((self.Kd * (error - self.prev_error)) / dt)
        self.prev_error = error
        
        
        command = self.proportional + self.integral + self.derivative
        return command
    
    def reset():
        pass

class Motor:
    def __init__(self, in1_pin, in2_pin, pwm_pin, stby_pin, encoder = None):
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
            
        #NOT CURRENTLY USING PID
        #self.pid = PID(3.0, 1.0, 0)
        
        self.max_duty = 90
    
    def drive_to_setpoint(self, setpoint, thresh=0.1, duty_magnitude=40):
        """
        pwm = self.pid.update(self.encoder.position, setpoint)
        if pwm != None:
            self.drive(pwm)
        """
        diff = self.encoder.position - setpoint
        print(self.encoder.position, diff)
        if abs(diff) <= thresh:
            self.brake()
            return True
        else:
            if self.encoder.position < setpoint:
                #forward
                self.drive(duty_magnitude)
            else:
                #reverse
                self.drive(-duty_magnitude)
        return False
            
        
    
    def drive(self, duty):
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
        
        
        
enc1 = QuadratureEncoder(4, 5, gear_ratio=75.0, cpr=12.0, polarity=-1)
motor1 = Motor(1, 2, 3, 0, enc1)

enc2 = QuadratureEncoder(9, 10, gear_ratio=75.0, cpr=12.0, polarity=1)
motor2 = Motor(6, 7, 8, 0, enc2)

enc3 = QuadratureEncoder(14, 15, gear_ratio=75.0, cpr=12.0, polarity=-1)
motor3 = Motor(11, 12, 13, 0, enc3)

cMotor1 = Motor(16, 17, 18, 22)
cMotor2 = Motor(19, 20, 21, 22)



motor1.encoder.position = 0.0
motor2.encoder.position = 0.0
motor3.encoder.position = 0.0


#TODO - tune effective length, better polarity

setpoint = 0
while True:
    i = input()
    if i == 'w':
        setpoint += 1
        while True:
            ret1 = motor1.drive_to_setpoint(-setpoint, thresh=0.05)
            ret2 = motor2.drive_to_setpoint(setpoint, thresh=0.05)
            ret3 = motor3.drive_to_setpoint(setpoint, thresh=0.05)
    
            if ret1 and ret2 and ret3:
                break
    elif i == 's':
        setpoint -= 1
        while True:
            ret1 = motor1.drive_to_setpoint(-setpoint, thresh=0.05)
            ret2 = motor2.drive_to_setpoint(setpoint, thresh=0.05)
            ret3 = motor3.drive_to_setpoint(setpoint, thresh=0.05)
    
            if ret1 and ret2 and ret3:
                break
    elif i == 'r':
        motor1.encoder.position = 0.0
        motor2.encoder.position = 0.0
        motor3.encoder.position = 0.0
    else:
        motor1.brake()
        motor2.brake()
        motor3.brake()


