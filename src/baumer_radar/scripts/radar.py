#!/usr/bin/env python

import rospy
from baumer_radar_msgs.msg import RadarData
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class FieldRobotNavigator:
    def __init__(self):
        rospy.init_node('field_robot_navigator')
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/filtered_data', RadarData, self.distance_callback)
        rospy.Subscriber('/teleop/automatic_mode', Bool, self.automatic_mode_callback)
        self.rate = rospy.Rate(10)

        self.actual_distance = 0
        self.min_dist = 0
        self.confidence=0
        self.automatic_mode = True
        self.shutdown_flag = False
        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):
        self.shutdown_flag = True

    def automatic_mode_callback(self,msg):
        self.automatic_mode = msg.data

    def distance_callback(self,msg):
        self.actual_distance = msg.distance
        self.confidence = msg.confidence
        if self.confidence > 0 and self.min_dist == 0: 
            self.min_dist=self.actual_distance
        rospy.loginfo("actual dist %f", self.actual_distance)

    def navigate(self):
        while not self.shutdown_flag:
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.3
            if  self.confidence > 0 and (self.min_dist-0.5)<self.actual_distance<(self.min_dist+0.5):
                cmd_vel.angular.z = (self.min_dist - self.actual_distance)
                rospy.loginfo("Gap to desired distance:%f",self.min_dist - self.actual_distance)
            else:
                cmd_vel.angular.z = 0
            rospy.loginfo("Publishing to cmd_vel: %s", cmd_vel)
            self.cmd_vel_pub.publish(cmd_vel)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        navigator = FieldRobotNavigator()
        navigator.navigate()
    except rospy.ROSInterruptException:
        pass