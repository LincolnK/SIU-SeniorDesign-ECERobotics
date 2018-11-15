#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan


laser_publisher = rospy.Publisher("scan",LaserScan,queue_size=10)


def cb(scan):
    newRanges = []
    n=0
    for i in scan.ranges:
        if(n%3==0):
            newRanges.append(scan.ranges[n])
        n+=1
    scan.ranges=newRanges
    laser_publisher.publish(scan)

    


def listen():
    rospy.init_node("scan_selector")
    rospy.Subscriber("raw_scan",LaserScan, cb)
    while not rospy.is_shutdown():
        rospy.spin()

if __name__=="__main__":
    listen()
