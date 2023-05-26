#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Twist, Point32


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
        points = point_cloud2.read_points(msg, skip_nans=True)
        points = np.array(list(points))
        
        # Filter points by x and y limits
        x_mask = np.logical_and(points[:, 0] > self.x_min, points[:, 0] < self.x_max)
        y_mask = np.logical_and(np.abs(points[:, 1]) > self.y_min, np.abs(points[:, 1]) < self.y_max)
        mask = np.logical_and(x_mask, y_mask)
        self.points = points[mask]
        
        # Convert points to Point32 format
        #points = [Point32(*p) for p in points]
        
        #self.points = points


    def navigate(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.points is None:
                rate.sleep()
                continue

            # Calculate the average distance to the robot on both sides within x and y limits
            left_y = [p.y for p in self.points if p.y < 0]
            right_y = [p.y for p in self.points if p.y >= 0]
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
                cmd_vel.angular.z = -5*center_dist
                cmd_vel.linear.x = 0.1
                rospy.loginfo("Publishing to cmd_vel: %s", cmd_vel)

            self.cmd_vel_pub.publish(cmd_vel)
            rate.sleep()


if __name__ == '__main__':
    navigator = FieldRobotNavigator()
    navigator.navigate()
