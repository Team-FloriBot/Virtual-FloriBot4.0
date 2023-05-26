#!/usr/bin/env python

from numpy import math
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class FieldRobotNavigator:
    def __init__(self):
        rospy.init_node('field_robot_navigator')

        # Set up subscribers and publishers
        rospy.Subscriber('laser_scanner_front', LaserScan, self.scan_callback)
        #self.sub_laser = rospy.Subscriber("sensors/scanFront", LaserScan, self.laser_callback_front, queue_size=3)   # [Laserscan] subscriber on /sensors/scanFront
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Initialize member variables
        self.robot_pose = None
        self.scan_data = None

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def scan_callback(self, msg):
        self.scan_data = msg.ranges
        self.angle_increment = msg.angle_increment

    def navigate(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.robot_pose is None or self.scan_data is None:
                rate.sleep()
                continue
            
            #print("Scan_Data", self.scan_data)
            # TODO: Implement navigation logic here
            # Convert the laser scan data from polar coordinates to Cartesian coordinates
            scan_x = []
            scan_y = []
            for i in range(len(self.scan_data)):
                if self.scan_data[i] > 0.1 and self.scan_data[i] < 10:  # Ignore invalid ranges
                    angle = self.robot_pose.orientation.z + (i - len(self.scan_data) / 2) * self.angle_increment * 3.1416 / 180.0
                    print("Angle", angle)
                    scan_x.append(self.scan_data[i] * math.cos(angle))
                    scan_y.append(self.scan_data[i] * math.sin(angle))
                    #print("Calculating cartesian")
            # Calculate the average distance to the robot on both sides within x and y limits
            left_sum_y = 0.0
            left_count = 0
            right_sum_y = 0.0
            right_count = 0
            x_min = -1.0  # Set the minimum x value for the left side
            x_max = 1.0 # Set the maximum x value for the left side
            for i in range(len(scan_x)):
                if scan_x[i] > x_min and scan_x[i] < x_max and abs(scan_y[i]) < 0.5:  # Use only points within x and y limits
                    left_sum_y += scan_y[i]
                    left_count += 1
            x_min = -1.0  # Set the minimum x value for the right side
            x_max = 1.0  # Set the maximum x value for the right side
            for i in range(len(scan_x)):
                if scan_x[i] > x_min and scan_x[i] < x_max and abs(scan_y[i]) < 0.5:  # Use only points within x and y limits
                    right_sum_y += scan_y[i]
                    right_count += 1
            
            if left_count == 0 or right_count == 0:
                # Not enough data to calculate center
                cmd_vel = Twist()
                print("At least at one side is no Maize")
            else:
                # Calculate the actual distance to the center of both sides
                left_dist = abs(left_sum_y / left_count)
                right_dist = abs(right_sum_y / right_count)
                center_dist = (left_dist + right_dist) / 2.0
                print("Distance to Center", center_dist)
            
                # Adjust the angular velocity to center the robot between the rows
                cmd_vel = Twist()
                print("Left:" + str(left_dist) + "Right:" + str(right_dist))
                if left_dist > right_dist:
                    cmd_vel.angular.z = -0.5
                    print("pub on cmd_vel")
                elif right_dist > left_dist:
                    cmd_vel.angular.z = 0.5
                    print("pub on cmd_vel")


            rate.sleep()

if __name__ == '__main__':
    navigator = FieldRobotNavigator()
    navigator.navigate()