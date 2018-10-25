#!/usr/bin/env python
from math import pi
#from collections import namedtuple
import contextlib
from sensor_msgs.msg import LaserScan
from ecer2.Sensor import Sensor
import rospy


#sensor_position = namedtuple('sensor_position',['x','y','angle'])
#fixed_sensor = namedtuple('sensor',['position','sensor'])


#from right, going counterclockwise
sensor_lines = []

last_time = None

def make_sensor_callback(sensors):
    publisher =  rospy.Publisher("ground_sensors",LaserScan, queue_size=10)
    def cb():
        global last_time
        current_time = rospy.Time.now()
        ranges = []
        msg = LaserScan()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'base_link'
        msg.angle_min = pi/6.
        msg.angle_max = 13. * pi/6.
        msg.angle_increment = pi/3.
        msg.time_increment = 0
        msg.scan_time = (current_time-last_time).to_sec()
        msg.range_min = 0.004
        msg.range_max = 0.05
        for s in sensors:
            try:
                ranges.append(float(s.value) * 0.03)
            except:
                pass
        msg.ranges = ranges
        publisher.publish(msg)
        last_time = current_time
    return cb


def run():
    global last_time
    sensor_objs = [Sensor(l) for l in sensor_lines]
    rospy.init_node('ground_sensors')
    last_time = rospy.Time.now()
    poll_rate = rospy.Rate(1) #poll once per second. have 
    with contextlib.nested(*sensor_objs) as sensors:
        cb = make_sensor_callback(sensors)
        for s in sensors:
            s.listen(cb)
        while not rospy.is_shutdown():
            cb()
            poll_rate.sleep()




if __name__ == "__main__":
    run()


