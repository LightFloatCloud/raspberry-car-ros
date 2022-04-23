# -*- coding: utf-8 -*-
# Import required libraries
import sys
import time
import RPi.GPIO as GPIO

# Use BCM GPIO references
# instead of physical pin numbers






# Define GPIO signals to use
# Physical pins 12,16,18,22 -> GPIO[18,23,24,25]
# [GPIO18:INT1, GPIO23:INT2, GPIO24:INT3, GPIO25:INT4]



class speed_count():
    left_edge_sum = 0
    right_edge_sum = 0
    speed = [0, 0]
    sample_time_inteval = 0.2

    def __init__(self, LeftPin=[27], RightPin=[22]):
        self.left  = LeftPin
        self.right = RightPin
        GPIO.setmode(GPIO.BCM)
        for pin in (self.left + self.right):
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            GPIO.add_event_detect(pin, GPIO.RISING, callback=self.my_callback )#在通道上添加上升临界值检测
        
        while True:
            #GPIO.wait_for_edge(12, GPIO.RISING)
            # GPIO.RISING    上升沿检测
            # GPIO.FALLING   下降沿检测
            # GPIO.BOTH      两者都可以，也就是说检测到边沿变化
            
            time.sleep(sample_time_inteval)
            speed = [left_edge_sum/sample_time_inteval, right_edge_sum/sample_time_inteval]
            left_edge_sum = 0
            right_edge_sum = 0 
            print("Recent speed:%s" %speed)
            
  

    def my_callback(self, channel):
        if(channel == self.left)
            left_edge_sum = left_edge_sum + 1
        elif(channel == self.right)
            right_edge_sum = right_edge_sum + 1
        
        
        print("在通道%s上进行边缘检测" %channel)

    def get_inteval(self):
        return sample_time_inteval
    
    def get_speed(self):
        return speed
        
if __function__ ==  __main__:
    
    speedcount_example = speed()
    
    
    
    
    
        