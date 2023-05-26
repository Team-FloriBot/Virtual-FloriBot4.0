#!/usr/bin/env python

import rospy  # Import the rospy module to write ROS nodes
import tf2_ros  # Import the tf2_ros module for transforming point clouds
from sensor_msgs.msg import PointCloud2  # Import the PointCloud2 message type
from geometry_msgs.msg import TransformStamped  # Import the TransformStamped message type
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud  # Import the do_transform_cloud function for transforming point clouds
#sudo apt-get install ros-noetic-tf2-sensor-msgs
import numpy as np  # Import the NumPy library for numerical computing
from sensor_msgs import point_cloud2 as pc2  # Import the point_cloud2 module for reading point clouds

class PointCloudMerger:
    def __init__(self):
        rospy.init_node('point_cloud_merger')  # Initialize the ROS node with the name 'point_cloud_merger'
        self.tf_buffer = tf2_ros.Buffer()  # Create a tf2_ros.Buffer object for storing transform information
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)  # Create a tf2_ros.TransformListener object for receiving transform information
        self.front_sub = rospy.Subscriber('/laser_scanner_front_cart', PointCloud2, self.front_callback)  # Subscribe to the '/laser_scanner_front_cart' topic and specify the callback function for processing received messages
        self.rear_sub = rospy.Subscriber('/laser_scanner_rear_cart', PointCloud2, self.rear_callback)  # Subscribe to the '/laser_scanner_rear_cart' topic and specify the callback function for processing received messages
        self.pub = rospy.Publisher('laser_scanner_combined_cart', PointCloud2, queue_size=10)  # Create a Publisher object for publishing the merged point cloud to the 'laser_scanner_combined_cart' topic with a queue size of 10

        self.front_cloud = None  # Initialize a variable to store the front point cloud
        self.rear_cloud = None  # Initialize a variable to store the rear point cloud

    def front_callback(self, front_cloud):
        self.front_cloud = front_cloud  # Store the received front point cloud message in the 'front_cloud' variable
        self.process_point_clouds()  # Call the 'process_point_clouds()' function to transform and merge the point clouds

    def rear_callback(self, rear_cloud):
        self.rear_cloud = rear_cloud  # Store the received rear point cloud message in the 'rear_cloud' variable
        self.process_point_clouds()  # Call the 'process_point_clouds()' function to transform and merge the point clouds

    def process_point_clouds(self):
        # Check if both point clouds have been received
        if self.front_cloud is None or self.rear_cloud is None:
            return

        try:
            # Get the transformation from the point cloud frame to the "base_link" frame
            front_transform = self.tf_buffer.lookup_transform('base_link', self.front_cloud.header.frame_id, self.front_cloud.header.stamp)
            rear_transform = self.tf_buffer.lookup_transform('base_link', self.rear_cloud.header.frame_id, self.rear_cloud.header.stamp)

            # Apply the transformations to the point clouds
            front_transformed_cloud = do_transform_cloud(self.front_cloud, front_transform)
            rear_transformed_cloud = do_transform_cloud(self.rear_cloud, rear_transform)

            # Merge the two point clouds into one
            merged_cloud = self.merge_point_clouds(front_transformed_cloud, rear_transformed_cloud)

            # Publish the merged point cloud
            self.pub.publish(merged_cloud)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # If there is an error with the transformations, log a warning and continue
            rospy.logwarn('Unable to transform point clouds: {}'.format(e))

        # Reset the point clouds
        self.front_cloud = None
        self.rear_cloud = None

    def merge_point_clouds(self, cloud1, cloud2):
        # Convert the point clouds to lists of points, merge the lists, and create a new point cloud
        merged_points = list(pc2.read_points(cloud1)) + list(pc2.read_points(cloud2))
        # Set the header for the merged point cloud
        merged_header = cloud1.header
        merged_header.frame_id = 'laser_scanner_combined_cart'
        # Create the merged point cloud message
        merged_cloud = pc2.create_cloud_xyz32(merged_header, merged_points)
        return merged_cloud
    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == '__main__':
    # Create an instance of the PointCloudMerger class and start the node
    node = PointCloudMerger()
    node.run()
