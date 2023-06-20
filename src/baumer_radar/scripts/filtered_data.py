#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range
from baumer_radar_msgs.msg import RadarData

last_valid_range = None


def range_callback(data):
    global last_valid_range

    if data.confidence > 0:
        # Publish the received range directly to the "filtered_data" topic
        filtered_range_pub.publish(data)
        last_valid_range = data
    elif data.confidence == 0 and last_valid_range is not None:
        # Publish the last valid range as long as there is no new range with a value of 0
        last_valid_range.confidence = data.confidence
        filtered_range_pub.publish(last_valid_range)


if __name__ == '__main__':
    rospy.init_node('range_filter', anonymous=True)
    rospy.Subscriber('decoded_radar_data', RadarData, range_callback)
    filtered_range_pub = rospy.Publisher('filtered_data', RadarData, queue_size=10)

    rospy.spin()



