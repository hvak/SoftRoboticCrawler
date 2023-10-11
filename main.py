from machine import Pin, PWM, Timer
import utime
from motor import *
        
enc1 = QuadratureEncoder(4, 5, gear_ratio=75.0, cpr=12.0, polarity=1)
motor1 = Motor(1, 2, 3, 0, enc1, polarity = -1)

enc2 = QuadratureEncoder(9, 10, gear_ratio=75.0, cpr=12.0, polarity=1)
motor2 = Motor(6, 7, 8, 0, enc2)

enc3 = QuadratureEncoder(14, 15, gear_ratio=75.0, cpr=12.0, polarity=-1)
motor3 = Motor(11, 12, 13, 0, enc3)

cMotor1 = Motor(16, 17, 18, 22)
cMotor2 = Motor(19, 20, 21, 22)


class StateMachine:
    def __init__(self):
        pass
    
    def run():
        pass

if __name__ == '__main__':
    st = StateMachine()

        
        
