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


totalOdom = numpy.array([0.0,0.0,0.0])
"""Current odometry estimate.  3 dimensional numpy array where:
    0: Estimated distance traveled in x axis
    1: Estmated distance traveled in y axis
    2: Estimated rotation around z axis
"""

coefficients = numpy.array([
    [max_linear_speed*-sin(pi/6),max_linear_speed*-sin(5*pi/6),max_linear_speed*-sin(3*pi/2)],
    [max_linear_speed*cos(pi/6),max_linear_speed*cos(5*pi/6),max_linear_speed*cos(3*pi/2)],
    [max_angular_speed,max_angular_speed,max_angular_speed]])
"""
Coefficient matrix used to calculate motor speeds. Calculated when node is first loaded

"""

last_velocities = numpy.array([0,0,0])
o_tf_broadcast = tf2_ros.TransformBroadcaster()
odom_publisher = rospy.Publisher("odom",Odometry,queue_size=50)
tf_buffer =tf2_ros.Buffer()



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




def publish_odom(current_velocities=None,first_time=False):
    """
        publishes current odometry estimate to tf and odom topics.

        Args:
            current_velocities (List[float],optional): updated velocity, if applicable. If not given, will use the last cached velocity estimate
    """
    global current_time
    global last_time
    global totalOdom
    global last_velocities
    global odom_publisher
    current_time = rospy.Time.now()
    dt = (current_time-last_time).to_sec()
    dist = last_velocities*float(dt)
    totalOdom[2]+=dist[2]#angular movement doesn't need to be changed tf frame
    quat = Quaternion(*quaternion_from_euler(0,0,totalOdom[2]))
    #Transform new odom position into world reference frame
    try:
        newPos=getNewPosition((dist[0],dist[1],0.))
        totalOdom[0]=newPos.point.x
        totalOdom[1]=newPos.point.y
    except Exception as e:
        print "error publishing odometry",e
        totalOdom[0]+=dist[0]
        totalOdom[1]+=dist[1]
    #tf2 library's odometry message
    t = tf2_ros.TransformStamped()
    t.header.stamp=rospy.Time.now()
    t.header.frame_id='odom'
    t.child_frame_id='base_link'
    t.transform.translation.x = totalOdom[0]
    t.transform.translation.y = totalOdom[1]
    t.transform.translation.z = 0.0
    t.transform.rotation = quat

    o_tf_broadcast.sendTransform(t)
    #Odom topic's message 
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "world"
    odom.pose.pose = Pose(Point(totalOdom[0], totalOdom[1], 0.), quat)
    odom.child_frame_id = "base_link"
    odom.twist.twist =  Twist(Vector3(last_velocities[0], last_velocities[1], 0.), Vector3(0., 0., last_velocities[2]))
    odom_publisher.publish(odom)
    if current_velocities is not None:
        last_velocities = current_velocities
    last_time = current_time




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
        targetVelocities = numpy.array([command.linear.y,command.linear.x,command.angular.z])
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
        publish_odom(targetVelocities)
    return callback



def listener():
    """
        Initialize and run main loop for the node. Sets up listeners for tf and cmd_vel
    """
    global current_time
    global last_time
    global tf_buffer
    try:
        rospy.init_node('base_controller')
        rospy.sleep(1)
        current_time = rospy.Time.now()
        last_time = rospy.Time.now()
        #print current_time
        #print last_time
        first_time=True
        publish_rate = rospy.Rate(10)#Publish odom info 4 times per second
        tf_buffer  = tf2_ros.Buffer()#Set up a tf listener so it's available for odom calculations
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        #Initialize motors. ContextManager for with block will handle initializing and freeing IO resources
        with Motor(20,21) as right, Motor(19,26) as left, Motor(6,13) as back:
            rospy.Subscriber("cmd_vel",Twist,build_control_callback(right,left,back))
            while not rospy.is_shutdown():
                publish_rate.sleep()
                try:
                    publish_odom(first_time=first_time)
                    first_time=False
                except Exception as e:
                    print "error in main loop",e
    except Exception as e:
        traceback.print_exc()

if __name__=="__main__":
    listener()

