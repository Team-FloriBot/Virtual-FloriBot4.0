#!/usr/bin/env python
#combined_frame_id = rospy.get_param('~combined_frame_id', 'front_laser')
#!/usr/bin/env python

import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs import point_cloud2

class PointCloudTransformer:
    def __init__(self):
        rospy.init_node('point_cloud_transformer')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pub_merged = rospy.Publisher('/merged_point_cloud', PointCloud2, queue_size=10)
        self.sub_front = rospy.Subscriber('/laser_scanner_front_cart', PointCloud2, self.transform_callback_front)
        self.sub_rear = rospy.Subscriber('/laser_scanner_rear_cart', PointCloud2, self.transform_callback_rear)
        self.merged_points = []
    
    def transform_callback_front(self, cloud_msg):
        try:
            transform = self.tf_buffer.lookup_transform('base_link', cloud_msg.header.frame_id, rospy.Time())
            transformed_cloud_msg = do_transform_cloud(cloud_msg, transform)
            transformed_cloud_msg.header.frame_id = 'base_link'
            self.merge_points(transformed_cloud_msg)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn('Error transforming point cloud: %s' % e)
    
    def transform_callback_rear(self, cloud_msg):
        try:
            transform = self.tf_buffer.lookup_transform('base_link', cloud_msg.header.frame_id, rospy.Time())
            transformed_cloud_msg = do_transform_cloud(cloud_msg, transform)
            transformed_cloud_msg.header.frame_id = 'base_link'
            self.merge_points(transformed_cloud_msg)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn('Error transforming point cloud: %s' % e)

    def merge_points(self, cloud_msg):
        for point in point_cloud2.read_points(cloud_msg, field_names=("x", "y", "z")):
            self.merged_points.append(point)
        merged_cloud = point_cloud2.create_cloud_xyz32(cloud_msg.header, self.merged_points)
        self.pub_merged.publish(merged_cloud)

if __name__ == '__main__':
    node = PointCloudTransformer()
    rospy.spin()
