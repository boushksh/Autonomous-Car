#!/usr/bin/env python3

import numpy as np
import math
import matplotlib.pyplot as plt
import rospy
from nav_msgs.msg import Odometry 
import rospy
from geometry_msgs.msg import Twist 

rospy.init_node('p_controller')


dt = 0.12 # [s] time tick
kp = 0.57 # Proportional Gain
speed_des = 3 #rospy.get_param("~speed_des_d")


#ROS Publisher for Velocity
pub = rospy.Publisher('/steer_bot/ackermann_steering_controller/cmd_vel',Twist,queue_size=10)

vel_x_msg = Twist() #Identify msg variable of data type Twist
rate = rospy.Rate(10) # rate of publishing msg 10hz

##Callback function for feedback the vehicle current position
#Callback function which is called when a new message of type Pose is received by the subscriber
def callback(data):
  global vel_x_msg	#Identify msg variable created as global variable
  global sub		#Identify a subscriber as global variable  
  vel_x_msg = data				#Initialize pos_msg with data sent to the function

sub = rospy.Subscriber('/steer_bot/gazebo/model_states', Twist, callback)

	
#Control function
def control():		#Identify alpha variable as global variable
   global linear_v	#Identify linear_v variable as global variable
   
   a = kp * (speed_des - vel_x_msg.linear.x)
   linear_v = vel_x_msg.linear.x +(a * dt)
   print("Velocity =" + str(linear_v))
   print("Acceleration =" + str(a))
   
vel_x_msg.linear.x = -20         #intial value of speed
pub.publish(vel_x_msg)
while 1 and not rospy.is_shutdown():
    control()
    vel_x_msg.linear.x = linear_v #Linear Velocity
    vel_x_msg.linear.y = 0
    vel_x_msg.linear.z = 0
    vel_x_msg.angular.x = 0
    vel_x_msg.angular.y = 0
    vel_x_msg.angular.z = 0 
    #ROS Code Publisher
    pub.publish(vel_x_msg)	#Publish msg
    rate.sleep()		#Sleep with rate
	
	
