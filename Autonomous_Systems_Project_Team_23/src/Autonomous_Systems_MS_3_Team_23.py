#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
import math


class LaneFollower:
    def __init__(self):
        self.sub = rospy.Subscriber('/steer_bot/ackermann_steering_controller/odom', Odometry, self.callback)
        self.pub = rospy.Publisher('/steer_bot/ackermann_steering_controller/cmd_vel', Twist, queue_size=100)
        self.goal = Point()
        self.goal.x = 0.0
        self.wheelbase = 0.2  # Wheelbase of the vehicle
        self.dt = 0.12  # [s] time tick
        self.kp = 0.57  # Proportional Gain

        #    Input       #
        self.speed_des = 0.8  # rospy.get_param("~speed_des_d")
        self.goal.y = -0.3 + 1 # lateral distance in the y direction (lane distance)

    def callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        # Compute the current position and heading of the robot
        current_pos = np.array([x, y])
        current_heading = np.array([math.cos(yaw), math.sin(yaw)])

        self.goal.x = current_pos[0] + 0.3

        # lookahead point calculation
        lookahead_point = [self.goal.x, self.goal.y]
        error = [self.goal.x, self.goal.y]

        # calculate error
        error[0] = lookahead_point[0] - current_pos[0]
        error[1] = lookahead_point[1] - current_pos[1]
        l_d = np.sqrt((error[0]) ** 2 + (error[1]) ** 2)

        # alpha calculations
        alpha = math.atan2(error[1], error[0]) - yaw

        # Compute the front steering angle using the Ackermann steering model
        steering_angle = math.atan((2 * self.wheelbase * np.sin(alpha)) / (l_d))
        steering_angle = np.interp(steering_angle, [-np.pi / 2, np.pi / 2], [-np.radians(30), np.radians(30)])

        # linear velocity control
        global linear_v  # Identify linear_v variable as global variable
        a = self.kp * (self.speed_des - msg.twist.twist.linear.x)
        linear_v = msg.twist.twist.linear.x + (a * self.dt)

        # Publish the control command to Gazebo
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_v
        cmd_vel.linear.y = 0
        cmd_vel.linear.z = 0
        cmd_vel.angular.x = 0
        cmd_vel.angular.y = 0
        cmd_vel.angular.z = steering_angle / linear_v

        print("Linear Velocity =" + str(msg.twist.twist.linear.x))
        print("Acceleration =" + str(a))
        print("Steering Angle =" + str(steering_angle))
        print("X =" + str(msg.pose.pose.position.x))
        print("Y =" + str(msg.pose.pose.position.y+0.3))
        self.pub.publish(cmd_vel)


if __name__ == '__main__':
    rospy.init_node('Autonomous_Systems_MS_3_Team_23')
    lf = LaneFollower()
    rospy.spin()
