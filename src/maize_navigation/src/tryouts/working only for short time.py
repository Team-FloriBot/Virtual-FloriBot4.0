#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Twist
import numpy as np


class FieldRobotNavigator:
    def __init__(self):
        rospy.init_node('field_robot_navigator')

        # Set up subscribers and publishers
        rospy.Subscriber('/merged_point_cloud', PointCloud2, self.point_cloud_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Initialize member variables
        self.robot_pose = None
        self.points = None

        # Initialize parameters
        self.x_min = rospy.get_param('x_min')
        self.x_max = rospy.get_param('x_max')
        self.y_min = rospy.get_param('y_min')
        self.y_max = rospy.get_param('y_max')


    def point_cloud_callback(self, msg):
        points = np.array(list(pc2.read_points(msg, skip_nans=True)))
        
        # Filter points by x and y limits
        mask = np.where(
            (points[:, 0] > self.x_min) & (points[:, 0] < self.x_max) &
            (np.abs(points[:, 1]) > self.y_min) & (np.abs(points[:, 1]) < self.y_max))[0]
        self.points = points[mask]

    def navigate(self):
        while not rospy.is_shutdown():
            if self.points is None:
                continue

            # Calculate the average distance to the robot on both sides within x and y limits
            left_y = self.points[self.points[:, 1] < 0, 1]
            right_y = self.points[self.points[:, 1] >= 0, 1]
            left_dist = np.mean(np.abs(left_y)) if len(left_y) > 0 else np.inf
            right_dist = np.mean(np.abs(right_y)) if len(right_y) > 0 else np.inf

            if np.isinf(left_dist) or np.isinf(right_dist):
                # Not enough data to calculate center
                cmd_vel = Twist()
            else:
                # Calculate the actual distance to the center of both sides
                center_dist = (right_dist - left_dist) / 2.0

                # Adjust the angular velocity to center the robot between the rows
                cmd_vel = Twist()
                cmd_vel.angular.z = -5*center_dist
                cmd_vel.linear.x = 0.1

            self.cmd_vel_pub.publish(cmd_vel)



if __name__ == '__main__':
    navigator = FieldRobotNavigator()
    navigator.navigate()

            # Convert points to Point32 format
        #points = [Point32(*p) for p in points]
        
        #self.points = points
