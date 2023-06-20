#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Point32
from std_msgs.msg import Bool


class FieldRobotNavigator:
    def __init__(self):
        rospy.init_node('field_robot_navigator')

        # Set up subscribers and publishers
        rospy.Subscriber('/merged_point_cloud', PointCloud2, self.point_cloud_callback)
        #self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.automatic_mode_pub = rospy.Publisher('/obstacle/obstacle_automatic_mode', Bool, queue_size=1)
        self.show_obstacle_pub = rospy.Publisher('/show_obstacle', Bool, queue_size=1)

        # Initialize member variables



        # Initialize parameters
        self.x_min_front = 0.1 #rospy.get_param('x_min_drive_in_row')
        self.x_max_front = 0.45 #rospy.get_param('x_max_drive_in_row')
        self.y_min_front = -0.08 #rospy.get_param('y_min_drive_in_row')
        self.y_max_front = 0.08 #rospy.get_param('y_max_drive_in_row')
        self.points_front = []
    def point_cloud_callback(self, msg):
        points_front = []

        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            point = Point32(*p)
            if self.y_min_front < np.abs(point.y) < self.y_max_front and self.x_min_front < point.x < self.x_max_front:
                points_front.append(point)
        self.points_front = points_front
        

    def navigate(self):
        rate = rospy.Rate(10)
        # Initialize previous values of spray_left and spray_right
        publish_once = False
        while not rospy.is_shutdown():
            if len(self.points_front) < 3:
            
                if publish_once:
                    rospy.loginfo("Now its free to drive..")
                    self.automatic_mode_pub.publish(True)
                    self.show_obstacle_pub.publish(False)
                    publish_once = False
                rate.sleep()
                continue
            else:
                if not publish_once:
                    rospy.loginfo("Path blocked...")
                    self.show_obstacle_pub.publish(True)
                    self.automatic_mode_pub.publish(False)
                    publish_once = True
          
                
             
if __name__ == '__main__':
    navigator = FieldRobotNavigator()
    navigator.navigate()
