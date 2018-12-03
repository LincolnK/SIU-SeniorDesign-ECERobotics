#! /usr/bin/env python

from math import asin,atan, sqrt,pi, sin, cos,acos
import rospy
import tf2_ros
import traceback
import numpy
from tf2_geometry_msgs import do_transform_point
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from geometry_msgs.msg import Twist, PoseStamped,Quaternion
from sensor_msgs.msg import LaserScan

GOAL_TOLERANCE=0.1
scan_data = None
current_goal = None
SPEED = 0.5
tf2_buffer = tf2_ros.Buffer()
tf2_listener = tf2_ros.TransformListener(tf2_buffer)
command_publisher = rospy.Publisher("cmd_vel",Twist, queue_size=10)
inf = float("inf")
angle_adjustment=None
last_pos=None
"""Difference between lidar and base_link"""

def getCurrentPoint():
    t = tf2_buffer.lookup_transform("map","base_link",rospy.Time(0),timeout=rospy.Duration(2))
    return numpy.array([t.transform.translation.x,t.transform.translation.y, t.transform.translation.z])

def goalReached(pos):
    if(current_goal is None):
        return True
    if( pos is None):
        return True
    if((abs(pos[0]-current_goal[0])<GOAL_TOLERANCE) and (abs(pos[1]-current_goal[1])<GOAL_TOLERANCE)):
        return True
    else:
        return False

def checkObstacle(angle,distance=1.5):
    adjusted=angle-pi/2
    def getScans(a,scan):
        n=int((a-scan.angle_min)/scan.angle_increment)
        num = len(scan.ranges)
        return (scan.ranges[n%num], scan.ranges[(n+1)%num],scan.intensities[n%num],scan.intensities[(n+1)%num], n)
    s = getScans(adjusted,scan_data)
    #print "checking angle",adjusted, "distances:", s[0],s[1], "intensities:", s[2],s[3],"n:",s[4], "num scans:", len(scan_data.ranges),"computed:", scan_data.angle_min+s[4]*scan_data.angle_increment
    return bool((s[0]<distance or s[1]<distance))# or (s[0]==inf and s[2]<1) or (s[1]==inf and s[3]<1)))


def sendCommand(angle):
    command=Twist()
    if angle is None:
        command.linear.x=0.
        command.linear.y=0.
    else:
        command.linear.x = SPEED* sin(angle)
        command.linear.y = SPEED*cos(angle)
        #print "sending angle ",angle, "(",command.linear.x,',',command.linear.y,')'
    command_publisher.publish(command)


def calc_command():
    global tf2_listener
    global last_pos
    vec = (0.,0.,0.)
    found_goal = False
    try:
        try:
            last_pos = getCurrentPoint()
        except:
            pass
        pos = last_pos
        if pos is None:
            return
        if goalReached(pos):
            sendCommand(None)
            return
        if scan_data is None:
            sendCommand(None)
            return
        #print "pos is ",pos
        #print "goal is", current_goal
        #dist = sqrt( (current_goal[0] - pos[0])**2 + (current_goal[1]-pos[1])**2)
        vec = current_goal-pos
        #vec = (current_goal[0]-pos[0],current_goal[1]-pos[1])
        #print "vector is",vec
        #begin by initializing an angle pointing straight towards goal
        #note. numpy.arctan2 takes care of x/y==x/0, and also quadrants
        left_angle = numpy.arctan2(vec[1], vec[2])
        #if(vec[1] == 0):
        #    left_angle = (3. if vec[0]<0 else 1.) * pi/2
        #else:
        #    left_angle = numpy.arctan2(vec[1],vec[0])#(pi if vec[1]<0 else 0) + atan(vec[0]/vec[1])
        #left_angle = (pi if pos[0]>current_goal[0] else 0) + (pi if pos[1]>current_goal[1]else 0)+ acos((current_goal[0]-pos[0])/dist)
        right_angle = left_angle
        #print "initial angle",left_angle
        increment = 0.015
        count = 0
        while not found_goal:
            #check each angle for an obstacle. If clear, we can use it
            if not checkObstacle(left_angle):
                sendCommand(left_angle)
                return
            if not checkObstacle(right_angle):
                sendCommand(right_angle)
                return
            #if blocked, try checking further left and right
            left_angle -= increment
            right_angle += increment
            count += 1
            #at first we only move a bit from the desired angle. With each failure we check further away
            increment += 0.005
            if count > 300: #too blocked in. Stop
                sendCommand(None)
                return
    except Exception as e:
        traceback.print_exc()



def listen_scan(data):
    global scan_data
    scan_data = data
    calc_command()

def listen_goal(data):
    global current_goal
    current_goal = numpy.array([data.pose.position.x,data.pose.position.y])
    calc_command()




def main_loop():
    global angle_adjustment
    rospy.init_node("greedy_nav")
    recalc_rate = rospy.Rate(2)
    while angle_adjustment is None:
        try:
            t = tf2_buffer.lookup_transform("base_link","lidar",rospy.Time())
            #print t.transform.rotation
            a = euler_from_quaternion([t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z, t.transform.rotation.w])
            angle_adjustment=a[2]
        except:
            traceback.print_exc()
            rospy.sleep(0.1)
    rospy.Subscriber("move_base_simple/goal",PoseStamped, listen_goal)
    rospy.Subscriber("scan",LaserScan,listen_scan)
    while not rospy.is_shutdown():
        calc_command()
        recalc_rate.sleep()


if __name__ == "__main__":
    main_loop()
