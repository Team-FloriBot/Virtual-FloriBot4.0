#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan 
#from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point32

class FieldRobotNavigator:
    def __init__(self):
        rospy.init_node('field_robot_navigator')

        # Set up subscribers and publishers
        rospy.Subscriber('laser_scanner_front', LaserScan, self.scan_callback)
        #rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Initialize member variables
        self.robot_pose = None
        self.scan_data = None
        self.angle_increment = None
        self.angle_min = None
        self.angle_max = None


    def scan_callback(self, msg):
        #self.scan_data = msg.ranges
        self.scan_data = np.array(msg.ranges)
        self.angle_increment = msg.angle_increment
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max

    def navigate(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.scan_data is None or self.angle_increment is None:
                rate.sleep()
                continue

            x_min = rospy.get_param('x_min')
            x_max = rospy.get_param('x_max')
            y_min = rospy.get_param('y_min')
            y_max = rospy.get_param('y_max')

            # Convert the laser scan data from polar coordinates to Cartesian coordinates
            mask = np.logical_and(self.scan_data > y_min, self.scan_data < 10)
            angles = np.arange(self.angle_min, self.angle_max + self.angle_increment, self.angle_increment)
            valid_angles = angles[mask]
            valid_ranges = self.scan_data[mask]
            scan_x = valid_ranges * np.cos(valid_angles)
            scan_y = valid_ranges * np.sin(valid_angles)

            # create a list of Point32 objects from the x and y values
            points_list = []
            for i in range(len(scan_x)):
                point = Point32()
                point.x = scan_x[i]
                point.y = scan_y[i]
                point.z = 0.0 # set the z value to 0, assuming a 2D plane
                points_list.append(point)


            # Calculate the average distance to the robot on both sides within x and y limits
            mask = np.logical_and(np.abs(scan_y) < y_max, np.logical_and(scan_x > x_min, scan_x < x_max))
            left_y = scan_y[mask & (scan_y < 0)]
            right_y = scan_y[mask & (scan_y >= 0)]
            left_dist = np.mean(np.abs(left_y)) if len(left_y) > 0 else np.inf #left is negative usually
            right_dist = np.mean(np.abs(right_y)) if len(right_y) > 0 else np.inf

            if np.isinf(left_dist) or np.isinf(right_dist):
                # Not enough data to calculate center
                cmd_vel = Twist()
                rospy.loginfo("At least one side has no maize")
            else:
                # Calculate the actual distance to the center of both sides
                center_dist = (right_dist - left_dist) / 2.0
                rospy.loginfo("Distance to center: %f", center_dist)

                # Adjust the angular velocity to center the robot between the rows
                cmd_vel = Twist()
                cmd_vel.angular.z = -2.5*center_dist
                cmd_vel.linear.x = 0.1
                rospy.loginfo("Publishing to cmd_vel: %s", cmd_vel)

            self.cmd_vel_pub.publish(cmd_vel)
            rate.sleep()

if __name__ == '__main__':
    navigator = FieldRobotNavigator()
    navigator.navigate()
