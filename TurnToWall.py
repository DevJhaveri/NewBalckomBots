import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from math import pi
from math import cos
from math import sin
from math import radians
from math import floor
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

def make_coor_array():
    with open('Coordinates.txt', 'r') as coordinates:
        coor_array = [tuple(map(float, line.split())) for line in coordinates]
    return coor_array

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

class Turn:
    def __init__(self, state, angle):
        if angle < 0:
            self.direction = 'R'
        else:
            self.direction = 'L'
        self.state = state
        self.target_angle = rectify_angle_pi(state.angle + angle)
        rospy.loginfo("Target angle: " + str(self.target_angle))
        self.done = False
        self.turning_angle = angle

    def act(self):
        # if still not at target, compute the next move to make.

        error = abs(self.target_angle - self.state.angle)
        rospy.loginfo("Current angle: " + str(self.state.angle))

        if (error > .02):
            if self.direction == 'L':
                move_cmd = Twist()
                move_cmd.angular.z = compute_vel_from_angle(error)
                self.state.cmd_vel.publish(move_cmd)
            elif self.direction == 'R':
                move_cmd = Twist()
                move_cmd.angular.z = -compute_vel_from_angle(error)
                self.state.cmd_vel.publish(move_cmd)
        else:
            self.state.cmd_vel.publish(Twist())
            self.done = True

    def __str__(self):
        print "Turned by " + str(self.turning_angle) + " radians"

rospy.init_node("Hough")
state = TurtlebotState()
array = state.make_laser_array()

def make_Hough_array():
    houghArray = []
    for i in xrange(0, 360):
        houghArray.append([0]*45)
    return houghArray

def fill_Hough_array(Hough_array, coordinate_array):
    for i in coordinate_array:
        x = i[0]
        y = i[1]
        for theta in xrange(0, 360):
            r = x*cos(radians(theta)) + y*sin(radians(theta))
            unrounded_index = r*9
            rounded_index = int(floor(unrounded_index))
            if rounded_index >= 0:
                Hough_array[theta][rounded_index] += 1
    return Hough_array

coor_array = make_coor_array()
array = make_Hough_array()
hough = fill_Hough_array(array, coor_array)
rate = rospy.Rate(20)

def turn_to_wall(Hough_array):
    for r in xrange(0, 45):
        for theta in xrange(0, 360):
            if Hough_array[theta][r] >= 100:
                action = Turn(state, radians(theta))
                while not action.done:
                    action.act()
                    rate.sleep()
