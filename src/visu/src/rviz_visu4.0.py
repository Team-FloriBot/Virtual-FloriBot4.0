#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def publish_bounding_boxes():
    rospy.init_node('bounding_box_publisher', anonymous=True)
    pub = rospy.Publisher('bounding_boxes', Marker, queue_size=10)
    rate = rospy.Rate(5)
    previous_box = 'initial'
    previous_sides = 'initial'
    rospy.set_param('box', 'drive')
    rospy.set_param('both_sides', 'both')
    rospy.loginfo("Start with default Drive BOX")
    idx=0
    while idx<=2:
        marker_left_msg = Marker()
        marker_left_msg.header.frame_id = "laserFront"
        marker_left_msg.ns = "bounding_boxes"
        marker_left_msg.id = 0  # ID of the left marker
        marker_left_msg.action = Marker.DELETE  # Set the action to DELETE
        marker_left_msg.lifetime = rospy.Duration(0)  # Set a short lifetime (0 seconds)

        marker_right_msg = Marker()
        marker_right_msg.header.frame_id = "laserFront"
        marker_right_msg.ns = "bounding_boxes"
        marker_right_msg.id = 1  # ID of the right marker
        marker_right_msg.action = Marker.DELETE  # Set the action to DELETE
        marker_right_msg.lifetime = rospy.Duration(0)  # Set a short lifetime (0 seconds)
        #rospy.sleep(0.1)  # Introduce a small delay before publishing deletion messages
        pub.publish(marker_left_msg)  # Publish deletion message for the left marker
        pub.publish(marker_right_msg)  # Publish deletion message for the right marker
        rospy.loginfo("Deleting old markers...")
        idx=idx+1
        rospy.sleep(0.1)
        

    while not rospy.is_shutdown():
        box = rospy.get_param('box')
        sides = rospy.get_param('both_sides')
        if sides != previous_sides or box != previous_box:
            if box == 'drive':
                x_min = rospy.get_param('x_min_drive_in_row')
                x_max = rospy.get_param('x_max_drive_in_row')
                y_min = rospy.get_param('y_min_drive_in_row')
                y_max = rospy.get_param('y_max_drive_in_row')
                #rospy.loginfo("1")
            elif box == 'exit':
                x_min = rospy.get_param('x_min_turn_and_exit')
                x_max = rospy.get_param('x_max_turn_and_exit')
                y_min = rospy.get_param('y_min_turn_and_exit')
                y_max = rospy.get_param('y_max_turn_and_exit')
            elif box == 'count':
                x_min = rospy.get_param('x_min_counting_rows')
                x_max = rospy.get_param('x_max_counting_rows')
                y_min = rospy.get_param('y_min_counting_rows')
                y_max = rospy.get_param('y_max_counting_rows')
            elif box == 'turn':
                x_min = rospy.get_param('x_min_turn_to_row')
                x_max = rospy.get_param('x_max_turn_to_row')
                y_min = rospy.get_param('y_min_turn_to_row')
                y_max = rospy.get_param('y_max_turn_to_row')
            elif box == 'turn_crit':
                x_min = rospy.get_param('x_min_turn_to_row_critic')
                x_max = rospy.get_param('x_max_turn_to_row_critic')
                y_min = rospy.get_param('y_min_turn_to_row_critic')
                y_max = rospy.get_param('y_max_turn_to_row_critic')
            else:
                rospy.loginfo("wrong box type")
            #rospy.loginfo("2")
            
            center_x = (x_min + x_max) / 2.0
            center_y = (y_min + y_max) / 2.0

            marker_left_msg = Marker()
            marker_left_msg.header.frame_id = "laserFront"
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
            marker_right_msg.header.frame_id = "laserFront"
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
            if sides == 'R':
                pub.publish(marker_right_msg)
                marker_left_msg.action = Marker.DELETE  # Set the action to DELETE
                pub.publish(marker_left_msg)
            elif sides == 'both':
                pub.publish(marker_right_msg)
                pub.publish(marker_left_msg)
                rospy.loginfo("Publish both Boxes")
            elif sides == 'L':
                pub.publish(marker_left_msg)
                marker_right_msg.action = Marker.DELETE  # Set the action to DELETE
                pub.publish(marker_right_msg)
            else:
                rospy.loginfo("no valid side")

            previous_box = box
            previous_sides = sides

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_bounding_boxes()
    except rospy.ROSInterruptException:
        pass
