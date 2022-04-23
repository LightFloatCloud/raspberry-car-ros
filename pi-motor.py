# Import required libraries
import sys
import time
import RPi.GPIO as GPIO

# Use BCM GPIO references
# instead of physical pin numbers


# Define GPIO signals to use
# Physical pins 12,16,18,22 -> GPIO[18,23,24,25]
# [GPIO18:INT1, GPIO23:INT2, GPIO24:INT3, GPIO25:INT4]

class motor():
    version = '0.01'
    speed = [0, 0]

    def __init__(self, LeftPin=[18, 23], RightPin=[24, 25]):
        self.left  = LeftPin
        self.right = RightPin
        GPIO.setmode(GPIO.BCM)
        for pin in (self.left + self.right):
            GPIO.setup(pin, GPIO.OUT)
        self.left_pwm  = [GPIO.PWM(self.left[0],  100), GPIO.PWM(self.left[1],  100)]
        self.right_pwm = [GPIO.PWM(self.right[0], 100), GPIO.PWM(self.right[1], 100)]

    def start(self):
        left_speed = self.speed[0]
        right_speed = self.speed[1]
        if left_speed > 0 :
            self.left_pwm[0].start(left_speed)
            self.left_pwm[1].start(0)
        else:
            self.left_pwm[0].start(0)
            self.left_pwm[1].start(-left_speed)
        if right_speed > 0 :
            self.right_pwm[0].start(right_speed)
            self.right_pwm[1].start(0)
        else:
            self.right_pwm[0].start(0)
            self.right_pwm[1].start(-right_speed)

    def stop(self):
        self.left_pwm[0].stop()
        self.left_pwm[1].stop()
        self.right_pwm[0].stop()
        self.right_pwm[1].stop()
 

    # speed: 0 ~ 100 
    def run_speed(self, left_speed, right_speed):
        self.speed[0] = left_speed 
        self.speed[1] = right_speed
    
        if left_speed > 0 :
            self.left_pwm[0].ChangeDutyCycle(left_speed)
            self.left_pwm[1].ChangeDutyCycle(0)
        else:
            self.left_pwm[0].ChangeDutyCycle(0)
            self.left_pwm[1].ChangeDutyCycle(-left_speed)
        if right_speed > 0 :
            self.right_pwm[0].ChangeDutyCycle(right_speed)
            self.right_pwm[1].ChangeDutyCycle(0)
        else:
            self.right_pwm[0].ChangeDutyCycle(0)
            self.right_pwm[1].ChangeDutyCycle(-right_speed)

if __function__ ==  __main__:
    
    m = motor()
    cmd_speed = int(input("Set speed:"))
    m.run_speed(cmd_speed, cmd_speed)
    m.start()
