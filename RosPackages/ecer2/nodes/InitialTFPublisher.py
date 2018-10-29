#!/usr/bin/env python
"""
    Static TF Publisher. Will publish the initial state of the robot at launch, then spin until shutdown
"""

import rospy
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist, Pose, Point, Quaternion


def getTransform(frame):
    """
        build the transform message between base_link and frame

        Args:
            frame (string): name of the frame

        Return:
            TransformStamped: A transform message ready to be sent out
    """
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
        #Static broadcaster can only send one transform. later calls overwrite
        #thus we need array so that we can publish everything at once.
        #this will matter more once things like the LIDAR frame are added
        tf_publisher.sendTransform([getTransform("world")])
        rospy.spin()