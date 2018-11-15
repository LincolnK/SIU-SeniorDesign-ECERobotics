#!/usr/bin/env python
r"""
BaseController translates overall movement goals into speed and direction outputs for the three motors. It does so by solving for the augmented matrix below, which was derived by applying an appropriate rotation matrix to the vector of each motor. It is intended to produce three numbers from -1 to 1, which are the duty cycle and direction of each motor.

If this results in a number outside those values, all will be scaled linearly to get a final output, which will preserve the direction, but not necessarily the speed requested. Using this scaled velocity, odometry will be calculated and published.


\\[
M= \\begin{bmatrix}
s_l \\cdot -sin(\\dfrac {pi}{6}) & s_l \\cdot -sin(\\dfrac {5pi}{6}) & s_l \\cdot -sin(\\dfrac {3pi}{2}) & V_x  \\\\\\\\
s_l \\cdot cos(\\dfrac {pi}{6}) & s_l \\cdot  cos(\\dfrac {5pi}{6}) & s_l \\cdot cos(\\dfrac {3pi}{2}) & V_y \\\\\\\\
s_a & s_a & s_a & V_{rz}
\\end{bmatrix}
\\]
"""

from math import sin,cos,pi
import traceback
import numpy
import rospy
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist,Pose,PointStamped,Point,Quaternion, Vector3,TransformStamped
from nav_msgs.msg import Odometry
from ecer2.Motor import Motor

max_linear_speed=2#temp
max_angular_speed=2#temp


m1angle=5*pi/6
m2angle=pi/3
m3angle=pi

coefficients = numpy.array([
    [max_linear_speed*-sin(m1angle),max_linear_speed*-sin(m2angle),max_linear_speed*-sin(m3angle)],
    [max_linear_speed*cos(m1angle),max_linear_speed*cos(m2angle),max_linear_speed*cos(m3angle)],
    [max_angular_speed,max_angular_speed,max_angular_speed]])
"""
Coefficient matrix used to calculate motor speeds. Calculated when node is first loaded

"""




def getNewPosition(point):
    """
        getNewPosition takes a position in the base_link's reference frame and transforms it into the world's frame of reference.

        Args:
            point (List[float]): x,y,rotation in base_link reference frame
    """
    global tf_buffer
    now = rospy.Time.now()
    check_time=rospy.Time(secs=now.secs-0.2)
    transform = tf_buffer.lookup_transform("odom","base_link",rospy.Time())
    p = PointStamped()
    p.point.x=point[0]
    p.point.y=point[1]
    newOdom = do_transform_point(p,transform)
    return newOdom








def build_control_callback(right, left, back):
    """
        Given three motor controllers, will return a callback function to be used when receiving a new velocity command

        Args:
            right (Motor): Controller for right motor
            left (Motor): Controller for left motor
            back (Motor): Controller for rear motor

        Returns:
            Callable: Callback to be used by ros topic subscriber when new velocity command is received
    """
    def callback(command):
        """
            Callback to be used to set motors when given a new velocity

            Args:
                command (geometry_msgs/Twist): Velocity command for robot
        """
        #3d vector. 0 is x velocity, 1 is y velocity, 2 is rotation about z axis
        targetVelocities = numpy.array([command.linear.x,command.linear.y,command.angular.z])
        #use numpy to solve system of equations defined by velocity vector and the coefficients above
        motorSpeeds = numpy.linalg.solve(coefficients,targetVelocities)
        #calculate the highest absolute value of a motor's speed.
        maxAbs = max(abs(motorSpeeds))
        if maxAbs>1:#Speed is a duty cycle, so it can only range from 0-100%. Must scale if higher
            motorSpeeds = motorSpeeds/maxAbs
            targetVelocities/=maxAbs
        motorSpeeds *=100.#pwm library expects 0-100 instead of 0-1
        right.speed = motorSpeeds[0]
        left.speed = motorSpeeds[1]
        back.speed = motorSpeeds[2]
    return callback



def listener():
    """
        Initialize and run main loop for the node. Sets up listeners for tf and cmd_vel
    """
    global current_time
    global last_time
    try:
        rospy.init_node('base_controller')
        current_time = rospy.Time.now()
        last_time = rospy.Time.now()
        #print current_time
        #print last_time
        #Initialize motors. ContextManager for with block will handle initializing and freeing IO resources
        with Motor(direction=20,pwm=21) as right, Motor(direction=26, pwm=19) as back, Motor(direction=16, pwm=12) as left:
            rospy.Subscriber("cmd_vel",Twist,build_control_callback(right,left,back))
            while not rospy.is_shutdown():
                rospy.spin()
    except Exception as e:
        traceback.print_exc()

if __name__=="__main__":
    listener()

