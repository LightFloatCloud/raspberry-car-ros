#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Import required libraries
import sys
import time
import RPi.GPIO as GPIO
import rospy
import math
# Use BCM GPIO references
# instead of physical pin numbers
from nav_msgs.msg  import Odometry

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

#import roslib
#roslib.load_manifest('learning_tf')

import tf
#import turtlesim.msg



# Define GPIO signals to use
# Physical pins 12,16,18,22 -> GPIO[18,23,24,25]
# [GPIO18:INT1, GPIO23:INT2, GPIO24:INT3, GPIO25:INT4]

# GPIO.BCM LeftPin=[27], RightPin=[22]

WHEEL_WIDTH = 0.123
WHEEL_LENGTH = 0.11

class speed_count():
    # Param Set
    pos = [0.0, 0.0, 0.0, 0.0] #x, y, z, theta
    speed = [0.0, 0.0]
    sample_time_freq = 10.0
    speed_scale = 1
    
    # Initialization
    rospy.init_node('velocity_publisher', anonymous=True)
    
    virtual_speed = rospy.get_param('virtual_speed', False)
    virtual_speed_value = rospy.get_param('virtual_speed_value', [0.1, 0.105])
    
    rate = rospy.Rate(sample_time_freq)
    odom_pos_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    
    
    left_edge_sum = 0
    right_edge_sum = 0
    sample_time_inteval = 1.0/sample_time_freq
    
    
    

    def __init__(self, LeftPin=[27], RightPin=[22]):
        self.left  = LeftPin
        self.right = RightPin
        GPIO.setmode(GPIO.BCM)
        for pin in (self.left + self.right):
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            GPIO.add_event_detect(pin, GPIO.RISING, callback=self.my_callback )#在通道上添加上升临界值检测
        
        while not rospy.is_shutdown():
            #GPIO.wait_for_edge(12, GPIO.RISING)
            # GPIO.RISING    上升沿检测
            # GPIO.FALLING   下降沿检测
            # GPIO.BOTH      两者都可以，也就是说检测到边沿变化
            
            #time.sleep(self.sample_time_inteval)
            if(self.virtual_speed == False):
                self.speed = [self.left_edge_sum/self.sample_time_inteval, self.right_edge_sum/self.sample_time_inteval]
                self.left_edge_sum = 0
                self.right_edge_sum = 0 
                
                self.speed[0] = self.speed[0]*self.speed_scale
                self.speed[1] = self.speed[1]*self.speed_scale
                
            else:
                self.speed = self.virtual_speed_value

            print("Recent speed:%s" %self.speed)
            self.pos_change()
            self.publishOdometry()
            self.rate.sleep()
  

    def my_callback(self, channel):
        if(channel == self.left[0]):
            self.left_edge_sum = self.left_edge_sum + 1
        elif(channel == self.right[0]):
            self.right_edge_sum = self.right_edge_sum + 1
        
        
        print("在通道%s上进行边缘检测" %channel)

    def get_inteval(self):
        return self.sample_time_inteval
    
    def get_speed(self):
        return self.speed
        
        
    def vel_move(self):
        return (self.speed[0]+self.speed[1])/2 
        
    def vel_turn(self):
        return (self.speed[1]-self.speed[0])/WHEEL_WIDTH
        
    def pos_change(self):
        v = self.vel_move()
        w = self.vel_turn()
        R = v / w * self.sample_time_inteval
        
        delta_theta = w * self.sample_time_inteval
        
        L = 2 * R * math.sin(delta_theta/2)
        
        delta_x = L * math.cos(self.pos[3] + v * self.sample_time_inteval/2)
        delta_y = L * math.sin(self.pos[3] + w * self.sample_time_inteval/2)
        
        self.pos[0] = self.pos[0] + delta_x
        self.pos[1] = self.pos[1] + delta_y
        self.pos[2] = self.pos[2] + 0
        self.pos[3] = self.pos[3] + delta_theta
        
    def publishOdometry(self):
        #def handle_pose(msg, carname):
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.pos[3])
        current_time = rospy.Time.now()
        br = tf.TransformBroadcaster()
        br.sendTransform((self.pos[0], self.pos[1], 0),
                     odom_quat,
                     current_time,
                     "base_link",
                     "odom")
        
        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(self.pos[0], self.pos[1], 0.), Quaternion(*odom_quat))
        
        odom.pose.covariance[0] = 0.1;
        odom.pose.covariance[7] = 0.1;
        odom.pose.covariance[14] = 0.1;
        odom.pose.covariance[21] = 1.0;
        odom.pose.covariance[28] = 1.0;
        odom.pose.covariance[35] = 1.0;

        # set the velocity
        odom.child_frame_id = "base_link"
        #odom.twist.twist = Twist(Vector3(self.vel_move(), 0, 0), Vector3(0, 0, self.vel_turn()))

        # publish the message
        self.odom_pos_pub.publish(odom)

        

if __name__ ==  '__main__':
    
    speedcount_example = speed_count()
    
    
