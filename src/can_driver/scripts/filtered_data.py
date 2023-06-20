#!/usr/bin/env python3

import rospy
import argparse
import numpy as np
from sensor_msgs.msg import Range
from baumer_radar_msgs.msg import RadarData


class RadarDataCollector:
    def __init__(self):
        self.time_data = []
        self.range_data = []
        self.velocity_data = []
        self.confidence_data = []

        self.filtered_range_pub = None

    def scan_callback(self, data):
        self.time_data.append(rospy.Time.now())
        self.range_data.append(data.distance)
        self.velocity_data.append(data.velocity)
        self.confidence_data.append(data.confidence)

    def run(self, duration, delay):
        rospy.init_node('radar_data_collector', anonymous=True)
        self.filtered_range_pub = rospy.Publisher('filtered_range_data', Range, queue_size=10)
        rospy.Subscriber('decoded_radar_data', RadarData, self.scan_callback)

        rospy.sleep(delay)  # Delay before starting data collection

        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()

            # Check if the specified duration has elapsed
            if (current_time - start_time).to_sec() >= duration:
                break

            # Only process data received after the delay
            duration_mask = [(t >= current_time - rospy.Duration(delay)) for t in self.time_data]
            range_data = [r for i, r in enumerate(self.range_data) if duration_mask[i]]

            # Apply running median filter to range data
            filtered_range_data_running_median = np.copy(range_data)
            window_size = 3  # Adjust as needed

            for i in range(window_size, len(filtered_range_data_running_median) - window_size):
                if filtered_range_data_running_median[i] == 0:
                    nonzero_indices = np.nonzero(filtered_range_data_running_median[i - window_size:i + window_size + 1])[0]
                    if len(nonzero_indices) > 0:
                        median_value = np.median(filtered_range_data_running_median[i - window_size:i + window_size + 1][nonzero_indices])
                        filtered_range_data_running_median[i] = median_value

            # Publish the filtered range data
            for i in range(len(filtered_range_data_running_median)):
                filtered_range_msg = Range()
                filtered_range_msg.header.stamp = rospy.Time.now()
                filtered_range_msg.range = filtered_range_data_running_median[i]
                self.filtered_range_pub.publish(filtered_range_msg)

            rospy.sleep(0.1)  # Adjust the sleep duration as needed

        rospy.loginfo(f"Collected {len(self.range_data)} range data points")

        rospy.spin()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Radar Data Collector')
    parser.add_argument('--duration', type=float, default=20, help='Duration in seconds')
    parser.add_argument('--delay', type=float, default=5, help='Delay in seconds')
    args = parser.parse_args()

    collector = RadarDataCollector()
    collector.run(args.duration, args.delay)




