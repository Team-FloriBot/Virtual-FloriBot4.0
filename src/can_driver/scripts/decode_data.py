#!/usr/bin/env python3

import rospy
from decimal import Decimal
from can_msgs.msg import Frame
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Range
from baumer_radar_msgs.msg import RadarData

def decode_can_message(can_msg):
    # Decode the data from the CAN message
    data = bytearray(can_msg.data)
    int_data = int.from_bytes(can_msg.data, byteorder='big')
    bit_data = ''.join(format(byte, '08b') for byte in can_msg.data)
    
    # Generate the used bitmasks
    sensor_status_mask = 0x0300000000000000
    confidence_value_mask = 0x00FF000000000000
    target_distance_mask = 0x0000FFFFFF000000
    not_used_mask = 0x0000000000FF0000
    target_velocity_mask = 0x000000000000FFFF
    
    # Extract the individual parts from the data
    sensor_status = (int_data & sensor_status_mask) >> 56
    confidence_value = (int_data & confidence_value_mask) >> 48
    target_distance_marker = (int_data & target_distance_mask) >> 24
    not_used = (int_data & not_used_mask) >> 16
    target_velocity_marker = (int_data & target_velocity_mask)
    
    # Modify the data to fit the estimatet results
    target_distance = int.from_bytes(target_distance_marker.to_bytes(3, byteorder='little'), byteorder='big')/10
    target_velocity = int.from_bytes(target_velocity_marker.to_bytes(2, byteorder='little'), byteorder='big') - 32768

    # Print the decoded data to the console
    #rospy.loginfo("Data: %s, Data7: %s, Data1: %s, Data4: %s, Sensor Status: 0x%X, Confidence Value: 0x%X, Target Distance: 0x%X, Not Used: 0x%X, Target Velocity:  0x%X", bit_data, data[7], data[1], data[4], sensor_status, confidence_value, target_distance, not_used, target_velocity)

    # Publish the decoded data as a new ROS message
    decoded_msg = RadarData()
    decoded_msg.status = sensor_status
    decoded_msg.confidence = confidence_value
    decoded_msg.distance = (target_distance/1000)
    decoded_msg.reserved = str(not_used)
    decoded_msg.velocity = (target_velocity/100)
    decoded_publisher.publish(decoded_msg)
    
    range_msg = Range()
    range_msg.header = can_msg.header
    range_msg.header.frame_id = 'radar_range'
    range_msg.radiation_type = 0
    range_msg.field_of_view = 0.174533
    range_msg.min_range = 0.4
    range_msg.max_range = 10
    range_msg.range = target_distance/1000
    range_publisher.publish(range_msg)
    


if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('can_message_decoder')

    # Create a subscriber to listen to CAN messages
    can_subscriber = rospy.Subscriber('received_messages', Frame, decode_can_message)

    # Create a publisher to publish the decoded data
    decoded_publisher = rospy.Publisher('decoded_radar_data', RadarData, queue_size=10)
    
    # Create a publisher to publish the range data
    range_publisher = rospy.Publisher('radar_range_data', Range, queue_size=10)
    
    # Spin the node so the subscriber and publisher can run
    rospy.spin()
