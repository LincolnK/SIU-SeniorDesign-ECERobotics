#!/usr/bin/env python 
import rospy
from geometry_msgs.msg import PoseStamped

publisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size = 5)

def publish_msg(command):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x = command[0]
        msg.pose.position.y = command[1]
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        publisher.publish(msg)

def publish_loop():
        i = 0
	rospy.init_node("test_commands")
	pb_rate = rospy.Rate(2.0)
        commands = [(3.75, 0.0), (0.0, 0.0)]
	while not rospy.is_shutdown():
                publish_msg(commands[(i/30)%2])
                pb_rate.sleep()
                i += 1
                
if(__name__ == "__main__"):
        publish_loop()
