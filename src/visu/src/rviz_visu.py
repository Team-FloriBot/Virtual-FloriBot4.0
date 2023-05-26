#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def publish_bounding_boxes():
    rospy.init_node('bounding_box_publisher', anonymous=True)
    pub = rospy.Publisher('bounding_boxes', Marker, queue_size=10)

    x_min = rospy.get_param('x_min')
    x_max = rospy.get_param('x_max')
    y_min = rospy.get_param('y_min')
    y_max = rospy.get_param('y_max')
    center_x = (x_min + x_max) / 2.0
    center_y = (y_min + y_max) / 2.0

    marker_left_msg = Marker()
    marker_left_msg.header.frame_id = "front_laser"
    marker_left_msg.type = Marker.CUBE
    marker_left_msg.action = Marker.ADD
    marker_left_msg.id = 0
    marker_left_msg.pose.position.x = center_x
    marker_left_msg.pose.position.y = -center_y
    marker_left_msg.pose.position.z = 0.0
    marker_left_msg.pose.orientation.x = 0.0
    marker_left_msg.pose.orientation.y = 0.0
    marker_left_msg.pose.orientation.z = 0.0
    marker_left_msg.pose.orientation.w = 1.0
    marker_left_msg.scale.x = abs(x_max - x_min)
    marker_left_msg.scale.y = abs(y_max - y_min)
    marker_left_msg.scale.z = 0.1
    marker_left_msg.color.a = 0.5
    marker_left_msg.color.r = 0.0
    marker_left_msg.color.g = 1.0
    marker_left_msg.color.b = 0.0

    marker_right_msg = Marker()
    marker_right_msg.header.frame_id = "front_laser"
    marker_right_msg.type = Marker.CUBE
    marker_right_msg.action = Marker.ADD
    marker_right_msg.id = 1
    marker_right_msg.pose.position.x = center_x
    marker_right_msg.pose.position.y = center_y
    marker_right_msg.pose.position.z = 0.0
    marker_right_msg.pose.orientation.x = 0.0
    marker_right_msg.pose.orientation.y = 0.0
    marker_right_msg.pose.orientation.z = 0.0
    marker_right_msg.pose.orientation.w = 1.0
    marker_right_msg.scale.x = abs(x_max - x_min)
    marker_right_msg.scale.y = abs(y_max - y_min)
    marker_right_msg.scale.z = 0.1
    marker_right_msg.color.a = 0.5
    marker_right_msg.color.r = 0.0
    marker_right_msg.color.g = 1.0
    marker_right_msg.color.b = 0.0

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(marker_left_msg)
        pub.publish(marker_right_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_bounding_boxes()
    except rospy.ROSInterruptException:
        pass
