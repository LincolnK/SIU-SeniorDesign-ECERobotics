"""
    ROS Node that sets up listeners for the ground facing IR sensors and publishes them as a LaserScan
"""
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
"""
    sensor_lines (List[int]): List of GPIO pins used for ground sensors
"""

last_time = None

def make_sensor_callback(sensors):
    """
        Given an array of sensors, will return a callback that publishes current sensor readings to ground_sensors topic

        Args:
            sensors (List[Sensor]): A list of Sensor objects. One for each sensor we want to read from

        Returns:
            Callable: Callback that polls the sensors and publishes a LaserScan message
    """
    publisher =  rospy.Publisher("ground_sensors",LaserScan, queue_size=10)
    def cb():
        global last_time
        current_time = rospy.Time.now()
        ranges = []
        msg = LaserScan()#Initialize blank message
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'base_link'
        msg.angle_min = pi/6. #rough estimate for position of first sensor
        msg.angle_max = 13. * pi/6. #2pi+ where we started
        msg.angle_increment = pi/3. #6 sensors around robot. Change if configuration changes
        msg.time_increment = 0 #used for rotating scanners. this one is simultaneous
        msg.scan_time = (current_time-last_time).to_sec() #current time
        msg.range_min = 0.004 #rough estimate on how far to the side sensors can detect
        msg.range_max = 0.05
        for s in sensors:#read each sensor and add to result
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
    poll_rate = rospy.Rate(1) #poll once per second. also adding interrupt handlers that will be called if it changes
    with contextlib.nested(*sensor_objs) as sensors:
        cb = make_sensor_callback(sensors)
        for s in sensors:
            s.listen(cb)
        while not rospy.is_shutdown():
            cb()
            poll_rate.sleep()




if __name__ == "__main__":
    run()


