#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Twist, Point32
from PIL import Image

class FieldRobotNavigator:
    def __init__(self):
        rospy.init_node('field_robot_navigator')

        # Set up subscribers and publishers
        rospy.Subscriber('/merged_point_cloud', PointCloud2, self.point_cloud_callback)
        #self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        #self.points_pub = rospy.Publisher('/field_points', PointCloud2, queue_size=1)

        # Initialize member variables
        self.robot_pose = None
        self.points_front = None
        self.points_side = None


        # Initialize parameters
        self.x_min_front = -0.1#rospy.get_param('x_min_drive_in_row')
        self.x_max_front = 0.25#rospy.get_param('x_max_drive_in_row')
        self.y_min_front = 0.1#rospy.get_param('y_min_drive_in_row')
        self.y_max_front = 0.75#rospy.get_param('y_max_drive_in_row')

        self.x_min_side = -0.2#rospy.get_param('x_min_drive_in_row')
        self.x_max_side = -0.1#rospy.get_param('x_max_drive_in_row')
        self.y_min_side = 0.1#rospy.get_param('y_min_drive_in_row')
        self.y_max_side = 0.75#rospy.get_param('y_max_drive_in_row')

    def point_cloud_callback(self, msg):
        points_front = []
        points_side=[]
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            point = Point32(*p)
            if self.y_min_front < np.abs(point.y) < self.y_max_front and self.x_min_front < point.x < self.x_max_front:
                points_front.append(point)
            if self.y_min_side < np.abs(point.y) < self.y_max_side and self.x_min_side < point.x < self.x_max_side:
                points_side.append(point)
        self.points_front = points_front
        self.points_side = points_side
        # Publish self.points
        #header = msg.header
        #header.frame_id = "front_laser"
        #fields = [PointField('x', 0, PointField.FLOAT32, 1),
        #          PointField('y', 4, PointField.FLOAT32, 1),
        #          PointField('z', 8, PointField.FLOAT32, 1)]
        #points = [(p.x, p.y, p.z) for p in self.points]
        #cloud = point_cloud2.create_cloud(header, fields, points)
        #self.points_pub.publish(cloud)


    def navigate(self):
        rate = rospy.Rate(10)
        # Initialize previous values of spray_left and spray_right
        prev_spray_left = 1
        prev_spray_right = 1
        spray_left=0
        spray_right=0
        spray_both_path = '/home/user/catkin_ws/src/spray/spray_pictures/spray_both.png'
        spray_right_path = '/home/user/catkin_ws/src/spray/spray_pictures/spray_R.png'
        spray_left_path = '/home/user/catkin_ws/src/spray/spray_pictures/spray_L.png'
        spray_not_path = '/home/user/catkin_ws/src/spray/spray_pictures/spray_not.png'

        while not rospy.is_shutdown():
            if self.points_front is None:
                rate.sleep()
                continue

            # Calculate the average distance to the robot on both sides within x and y limits
            front_left_y = [p.y for p in self.points_front if p.y < 0]
            front_right_y = [p.y for p in self.points_front if p.y >= 0]
            side_left_y = [p.y for p in self.points_side if p.y < 0]
            side_right_y = [p.y for p in self.points_side if p.y >= 0]

            if len(side_left_y)>0:
                spray_left = 1
            else:
                if len(front_left_y)==0:
                    spray_left =0

            if len(side_right_y)>0:
                spray_right = 1
            else:
                if len(front_right_y)==0:
                    spray_right = 0

           # Check if spray_left or spray_right has changed
            if spray_left != prev_spray_left or spray_right != prev_spray_right:
                if spray_left == 1 and spray_right == 1:
                    image = Image.open(spray_both_path)
                elif spray_left == 0 and spray_right == 1:
                    image = Image.open(spray_right_path)
                elif spray_left == 1 and spray_right == 0:
                    image = Image.open(spray_left_path)
                else:
                    image = Image.open(spray_not_path)
                #image.close()    
                image.show()
            # Update previous values
            prev_spray_left = spray_left
            prev_spray_right = spray_right
             
if __name__ == '__main__':
    navigator = FieldRobotNavigator()
    navigator.navigate()
