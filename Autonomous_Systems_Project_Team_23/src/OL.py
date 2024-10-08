#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry 

rospy.init_node('OL')

pub = rospy.Publisher('/steer_bot/ackermann_steering_controller/cmd_vel',Twist,queue_size=10)
pub1 = rospy.Publisher('/steer_bot/ackermann_steering_controller/odom',Odometry,queue_size=10)

while not rospy.is_shutdown():
 msg = Twist()
 msg.linear.x = rospy.get_param("~x_d")
 msg.angular.z = rospy.get_param("~w_d")
 
 msg1=Odometry()
 
 msg1.pose.pose.position.x = rospy.get_param("~px_d")
 msg1.pose.pose.position.y = rospy.get_param("~py_d")
 msg1.pose.pose.position.z = rospy.get_param("~pz_d") #always zero
 
 msg1.pose.pose.orientation.x = rospy.get_param("~ox_d")
 msg1.pose.pose.orientation.y = rospy.get_param("~oy_d")
 msg1.pose.pose.orientation.z = rospy.get_param("~oz_d")
 
 print("Linear-X = " + str(msg.linear.x),"Angular-Z = " + str(msg.angular.z))
 print("X-pos = " + str(msg1.pose.pose.position.x),"Y-pos = " + str(msg1.pose.pose.position.y))
 print("Orient-X = " + str(msg1.pose.pose.orientation.x),"Orient-Y = " + str(msg1.pose.pose.orientation.y),"Orient-Z = " + str(msg1.pose.pose.orientation.z))

 pub.publish(msg)
 pub1.publish(msg1)
 
