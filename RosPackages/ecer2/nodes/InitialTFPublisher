#!/usr/bin/env python

import rospy
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist, Pose, Point, Quaternion


def getTransform(frame):
    t = tf2_ros.TransformStamped()
    t.header.stamp=rospy.Time.now()
    t.header.frame_id=frame
    t.child_frame_id="base_link"
    t.transform.rotation = Quaternion(*quaternion_from_euler(0,0,0))
    return t


if __name__=="__main__":
    rospy.init_node('command_generator')
    while not rospy.is_shutdown():
        tf_publisher = tf2_ros.StaticTransformBroadcaster()
        tf_publisher.sendTransform([getTransform("world")])
        rospy.spin()
