#!/usr/bin/env python
#combined_frame_id = rospy.get_param('~combined_frame_id', 'front_laser')
#!/usr/bin/env python

import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs import point_cloud2

import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as point_cloud2


import rospy
import tf2_ros
import sensor_msgs.point_cloud2 as point_cloud2
from sensor_msgs.msg import PointCloud2


class PointCloudTransformer:
    def __init__(self):
        rospy.init_node('point_cloud_transformer')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pub_merged = rospy.Publisher('/merged_point_cloud', PointCloud2, queue_size=10)
        self.sub_front = rospy.Subscriber('/laser_scanner_front_cart', PointCloud2, self.transform_callback_front)
        self.sub_rear = rospy.Subscriber('/laser_scanner_rear_cart', PointCloud2, self.transform_callback_rear)
        self.front_points = None
        self.rear_points = None
        self.current_header = None
    
    def transform_callback_front(self, cloud_msg):
        try:
            merge_frame = rospy.get_param('~merge_frame', 'front_laser')
            transform = self.tf_buffer.lookup_transform(merge_frame, cloud_msg.header.frame_id, rospy.Time())
            transformed_cloud_msg = do_transform_cloud(cloud_msg, transform)
            transformed_cloud_msg.header.frame_id = merge_frame
            self.front_points = transformed_cloud_msg
            self.check_and_publish()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn('Error transforming point cloud: %s' % e)
    
    def transform_callback_rear(self, cloud_msg):
        try:
            merge_frame = rospy.get_param('~merge_frame', 'front_laser')
            transform = self.tf_buffer.lookup_transform(merge_frame, cloud_msg.header.frame_id, rospy.Time())
            transformed_cloud_msg = do_transform_cloud(cloud_msg, transform)
            transformed_cloud_msg.header.frame_id = merge_frame
            self.rear_points = transformed_cloud_msg
            self.check_and_publish()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn('Error transforming point cloud: %s' % e)
    
    def check_and_publish(self):
        if self.front_points is not None and self.rear_points is not None:
            self.merge_points()
            merged_cloud = point_cloud2.create_cloud_xyz32(self.current_header, self.merged_points)
            self.pub_merged.publish(merged_cloud)
            self.merged_points = []
            self.front_points = None
            self.rear_points = None
    
    def merge_points(self):
        merged_points = []
        for point in point_cloud2.read_points(self.front_points, field_names=("x", "y", "z")):
            merged_points.append(point)
        for point in point_cloud2.read_points(self.rear_points, field_names=("x", "y", "z")):
            merged_points.append(point)
        self.current_header = self.front_points.header
        self.merged_points = merged_points


if __name__ == '__main__':
    node = PointCloudTransformer()
    rospy.spin()