#!/usr/bin/env python
"""
    Static TF Publisher. Will publish the initial state of the robot at launch, then spin until shutdown
"""
from math import pi
import rospy
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, PoseWithCovarianceStamped


def getTransform(parentFrame,childFrame,angle=(0,0,0)):
    """
        build the transform message between a parent and child frame, where both are initially identical in position

        Args:
            parentFrame (string): name of the parent frame
            childFrame (string): name of the child frame

        Return:
            TransformStamped: A transform message ready to be sent out
    """
    t = tf2_ros.TransformStamped()
    t.header.stamp=rospy.Time.now()
    t.header.frame_id=parentFrame
    t.child_frame_id=childFrame
    t.transform.rotation = Quaternion(*quaternion_from_euler(*angle))
    return t


if __name__=="__main__":
    rospy.init_node('static_publisher')
    while not rospy.is_shutdown():
        tf_publisher = tf2_ros.StaticTransformBroadcaster()
        #Static broadcaster can only send one transform. later calls overwrite
        #thus we need array so that we can publish everything at once.
        #this will matter more once things like the LIDAR frame are added
        tf_publisher.sendTransform([getTransform('map','odom'),getTransform("world","map"),getTransform("base_link","lidar",(0,0,pi))])
        initialPose = PoseWithCovarianceStamped()
        initialPose.header.stamp = rospy.Time.now()
        initialPose.header.frame_id="map"
        initialPose.pose.pose.position.x=0.0
        initialPose.pose.pose.position.y=0.0
        initialPose.pose.pose.position.z=0.
        initialPose.pose.pose.orientation.x=0.
        initialPose.pose.pose.orientation.y=0.
        initialPose.pose.pose.orientation.z=0.
        initialPose.pose.pose.orientation.w=1.0
        posePublisher = rospy.Publisher("initialpose",PoseWithCovarianceStamped,latch=True,queue_size=12)
        posePublisher.publish(initialPose)

        rospy.spin()
