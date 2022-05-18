# Import required libraries
import sys
import time
import RPi.GPIO as GPIO

import rospy
from geometry_msgs.msg import Twist

# Use BCM GPIO references
# instead of physical pin numbers

WHEEL_WIDTH = 0.123
WHEEL_LENGTH = 0.11
# Define GPIO signals to use
# Physical pins 12,16,18,22 -> GPIO[18,23,24,25]
# [GPIO18:INT1, GPIO23:INT2, GPIO24:INT3, GPIO25:INT4]

class motor():
    version = '0.01'
    speed = [0.0, 0.0] # speed值域：0~100
    PWM_frequency = 10.0
    
    drive_twist2vel_scale = 100
    drive_twist2ang_scale = 1 * WHEEL_WIDTH/2
    
    # 初始化 ros 节点
    rospy.init_node("Twist_Driver", anonymous=True)

    # 初始化控制命令发布者
    cmd_vel_pub = rospy.Subsciber('cmd_vel', Twist, twist_process_CallBack, queue_size=2)
    

    def __init__(self, LeftPin=[18, 23], RightPin=[24, 25]):
        self.left  = LeftPin
        self.right = RightPin
        GPIO.setmode(GPIO.BCM)
        for pin in (self.left + self.right):
            GPIO.setup(pin, GPIO.OUT)
        self.left_pwm  = [GPIO.PWM(self.left[0],  self.PWM_frequency), GPIO.PWM(self.left[1],  self.PWM_frequency)]
        self.right_pwm = [GPIO.PWM(self.right[0], self.PWM_frequency), GPIO.PWM(self.right[1], self.PWM_frequency)]

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
    def run_speed(self, left_speed, right_speed): # speed值域：0~100
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
    def twist_process_CallBack(msg):
        rospy.loginfo("Base vel: x:%06f, y:%0.6f, ang: z:%0.6f",\
            msg.linear.x, msg.linear.y, msg.angular.z)
        
        self.speed[0] = (msg.linear.x - drive_twist2ang_scale*msg.angular.z) * drive_twist2vel_scale
        self.speed[1] = (msg.linear.x + drive_twist2ang_scale*msg.angular.z) * drive_twist2vel_scale
        if self.speed[0]>100 :
            self.speed[0] = 100
        elif self.speed[0]<-100 :
            self.speed[0] = -100
        
        if self.speed[1]>100 :
            self.speed[1] = 100
        elif self.speed[1]<-100 :
            self.speed[1] = -100
            
        self.start()


            
    

def main():
    TestState = True
    m = motor()
    
    while TestState:
        cmd_speed = int(input("Set speed:"))
        m.run_speed(cmd_speed, cmd_speed)
        m.start()
        
    rospy.spin()

if __name__ ==  '__main__':
    main()
