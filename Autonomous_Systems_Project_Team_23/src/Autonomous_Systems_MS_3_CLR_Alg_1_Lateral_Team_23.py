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
        self.pub = rospy.Publisher('/steer_bot/ackermann_steering_controller/cmd_vel', Twist, queue_size=10)
        self.goal = Point()
        self.goal.x = 0.0  # Set the goal position in the x-direction
        self.goal.y = -0.15 + -1   # lateral distance in the y direction (lane distance)
        self.wheelbase = 0.2  # Wheelbase of the vehicle

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
        alpha = - yaw + math.atan2(error[1], error[0])

        # Compute the front steering angle using the Ackermann steering model
        steering_angle = math.atan((2 * self.wheelbase * np.sin(alpha)) / l_d)
        steering_angle = np.interp(steering_angle, [-np.pi / 2, np.pi / 2], [-np.radians(30), np.radians(30)])

        # Publish the control command to Gazebo
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.8 # Constant forward velocity
        cmd_vel.angular.z = steering_angle/cmd_vel.linear.x
        print("Linear Velocity =" + str(cmd_vel.linear.x))
        print("Steering Angle =" + str(steering_angle))
        print("X =" + str(msg.pose.pose.position.x))
        print("Y =" + str(msg.pose.pose.position.y))
        self.pub.publish(cmd_vel)


if __name__ == '__main__':
    rospy.init_node('Autonomous_Systems_MS_3_CLR_Alg_1_Lateral_Team_23')
    lf = LaneFollower()
    rospy.spin()
