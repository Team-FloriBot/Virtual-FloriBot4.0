#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs
import std_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()

    #rospy.wait_for_service('spawn')
    #spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    #spawner(4, 2, 0, 'turtle2')

    body_angle = rospy.Publisher('sensors/body_angle', std_msgs.msg.Float64,queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/jointRear', '/jointFront', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        quaternion = (rot[0],rot[1],rot[2],rot[3])

        euler = tf.transformations.euler_from_quaternion(quaternion)

        angle = euler[2]
        
        body_angle.publish(angle)

        rate.sleep()
