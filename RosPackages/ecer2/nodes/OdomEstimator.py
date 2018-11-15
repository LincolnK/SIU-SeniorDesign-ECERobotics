#!/usr/bin/env python

from math import cos, sin, pi
import tf2_ros
import rospy
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist,Pose, PointStamped, Point, Quaternion, Vector3, TransformStamped
from nav_msgs.msg import Odometry
import numpy

max_linear_speed=2
max_angular_speed=2


totalOdom = numpy.array([0.0,0.0])
"""Current odometry estimate.  3 dimensional numpy array where:
    0: Estimated distance traveled in x axis
    1: Estmated distance traveled in y axis
"""
totalAngle = 0.0

last_velocities = numpy.array([0,0,0])
odom_publisher = rospy.Publisher("odom",Odometry,queue_size=3)
tf_broadcast = tf2_ros.TransformBroadcaster(queue_size=20)


def publish_odom(current_velocities=None):
    """
        publishes current odometry estimate to tf and odom topics.

        Args:
            current_velocities (List[float],optional): updated velocity, if applicable. If not given, will use the last cached velocity estimate
    """
    global current_time
    global last_time
    global totalOdom
    global totalAngle
    global last_velocities
    global odom_publisher
    global tf_broadcast
    current_time = rospy.Time.now()
    dt = (current_time-last_time).to_sec()
    dist = last_velocities*float(dt)
    #totalOdom[2]+=dist[2]#angular movement doesn't need to be changed tf frame
    transform = numpy.matrix([[cos(totalAngle),-sin(totalAngle)],[sin(totalAngle),cos(totalAngle)]])
    linear = numpy.matrix(dist[:2]).transpose()
    linear = transform * linear
    totalOdom[0]+=linear[0,0]
    totalOdom[1]+=linear[1,0]
    totalAngle+=dist[2]
    #tf2 library's odometry message
    quat = Quaternion(*quaternion_from_euler(0,0,totalAngle))
    t = tf2_ros.TransformStamped()
    t.header.stamp=rospy.Time.now()
    t.header.frame_id='odom'
    t.child_frame_id='base_link'
    t.transform.translation.x = totalOdom[0]
    t.transform.translation.y = totalOdom[1]
    t.transform.translation.z = 0.0
    t.transform.rotation = quat

    tf_broadcast.sendTransform(t)
    #Odom topic's message 
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "world"
    odom.pose.pose = Pose(Point(totalOdom[0], totalOdom[1], 0.), quat)
    odom.child_frame_id = "base_link"
    odom.twist.twist =  Twist(Vector3(last_velocities[0], last_velocities[1], 0.), Vector3(0., 0., last_velocities[2]))
    odom_publisher.publish(odom)
    last_time = current_time
    if current_velocities is not None:
        last_velocities = current_velocities

def update_velocity(v):
    next_vel = numpy.array([v.linear.x,v.linear.y, v.angular.z])
    publish_odom(next_vel)


def listener():
    global current_time
    global last_time
    rospy.init_node('odom_estimator')
    current_time=rospy.Time.now()
    last_time=rospy.Time.now()
    publish_rate = rospy.Rate(20)#Publish odom info  times per second
    rospy.Subscriber("cmd_vel",Twist,update_velocity)
    while not rospy.is_shutdown():
        try:
            publish_odom()
            #rospy.sleep(publish_rate)
            publish_rate.sleep()
        except Exception as e:
            print e

if __name__=="__main__":
    listener()
