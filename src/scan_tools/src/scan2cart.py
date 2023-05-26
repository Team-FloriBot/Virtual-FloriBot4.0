#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection

class LaserScanToPointCloud:
    def __init__(self):
        # Initialize the node
        rospy.init_node('laser_scan_to_pointcloud')

        # Set default topic names for subscriber and publisher
        scan_topic = rospy.get_param('~scan_topic', '/laser_scanner_front')
        pointcloud_topic = rospy.get_param('~pointcloud_topic', '/laser_scanner_front_cart')
        
        # Create a LaserProjection object for converting laser scans to point clouds
        self.lp = LaserProjection()
        
        # Subscribe to the laser scan topic
        self.sub = rospy.Subscriber(scan_topic, LaserScan, self.callback)
        
        # Publish the point cloud topic
        self.pub = rospy.Publisher(pointcloud_topic, PointCloud2, queue_size=10)


    def callback(self, data):
        # Convert the laser scan data to a point cloud
        cloud = self.lp.projectLaser(data)
        
        # Publish the point cloud data
        self.pub.publish(cloud)

    def run(self):
        # Keep the node running until it is stopped
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == '__main__':
    # Create an instance of the LaserScanToPointCloud class and run the node
    node = LaserScanToPointCloud()
    node.run()
