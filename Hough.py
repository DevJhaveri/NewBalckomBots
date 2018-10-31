import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from math import pi
from math import cos
from math import sin
from numpy.linalg import norm
from numpy import array
from numpy.linalg import inv
import numpy as np
from angles import rectify_angle_pi
from sensor_msgs.msg import LaserScan

def yaw_from_odom(msg):
    orientation_q = msg.pose.pose.orientation
    orientation_vec = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_vec)

    return yaw

class TurtlebotState:
    def __init__(self):
        # start up the subscribers to monitor state

        self.subscriber_odom = rospy.Subscriber("/odom", Odometry, self.update_odom)
        self.laser_msg = rospy.Subscriber("/scan", LaserScan, self.update_laser)
        self.x = None
        self.y = None
        self.angle = None
        self.ranges = None
        self.scan = None
        self.anginc = None
        self.anglestart = None

        self.angle_max = None
        self.rmax = None
        self.rmin = None
        #ready indicates whether the odometry is received.
        self.odomReady = False
        self.laserReady = False

        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd_scan = rospy.Publisher('/scan', LaserScan, queue_size=1)


        # wait until odometry received, etc before
        # returning control
        while not self.odomReady or not self.laserReady:
            rate = rospy.Rate(20)
            rate.sleep()

    def update_laser(self, scan):
        rospy.loginfo("Scan done")
        self.ranges = list(scan.ranges)
        self.anginc = scan.angle_increment
        self.anglestart = scan.angle_min 
        self.angle_max = scan.angle_max
        self.rmax = scan.range_max
        self.rmin = scan.range_min
        self.laserReady = True

    def update_odom(self, msg):
        self.angle = yaw_from_odom(msg)
        #print (str(msg.pose.pose.position))
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.odomReady = True

    def make_laser_array(self):
        laser_array = []
        for i in xrange(len(self.ranges)):
            range = self.ranges[i]
            if range != 0:
                angle = i * self.anginc + self.angle
                angle = rectify_angle_pi(angle)
                x = range * cos(angle) + self.x
                y = range * sin(angle) + self.y
                laser_array.append((x, y))
        return laser_array

    def shutdown(self):
        rospy.loginfo("Shutting down turtlebot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
        rospy.loginfo("Goodbye.")

rospy.init_node("Hough")
state = TurtlebotState()
array = state.make_laser_array()
print(array)
print(len(array))

file = open("Coordinates.txt", 'w')
file.write('\n'.join('{} {}'.format(i[0],i[1]) for i in array))

