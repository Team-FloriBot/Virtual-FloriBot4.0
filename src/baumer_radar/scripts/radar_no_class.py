#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from baumer_radar_msgs.msg import RadarData
from sensor_msgs import point_cloud2
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Point32

actall_distance =0
confidence =0
min_dist = 0
cmd_vel = Twist()
init = True


def automatic_mode_callback(msg):
# Callback function to process the received message
       automatic_mode = msg.data

def distance_callback(msg):
       print("in cb")
       global min_dist
       global init
       actual_distance = msg.distance
       confidence = msg.confidence
       if confidence > 0 and init:
              min_dist =actual_distance
              init = False
              cmd_vel.linear.x = 0.3
       if  confidence > 0 and (min_dist-0.35)<actual_distance<(min_dist+0.35):
        # Compute the shortest y distance
              cmd_vel.angular.z = 5*(min_dist - actual_distance)
              rospy.loginfo("Gap to desired distance:%f",min_dist - actual_distance)
       else:
              cmd_vel.angular.z = 0
       rospy.loginfo("Publishing to cmd_vel: %s", cmd_vel)
       cmd_vel_pub.publish(cmd_vel)


if __name__ == '__main__':
       init = True
       rospy.init_node('field_robot_navigator')
       rospy.Subscriber('/filtered_data', RadarData, distance_callback)
       rospy.Subscriber('/teleop/automatic_mode', Bool, automatic_mode_callback)
       cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
       
       rospy.spin()

    #navigator.navigate()
