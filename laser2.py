"""Turn function for CS81 HW3.
Team Members: Andrew Sosanya, Anne Sherrill, Dev Jhaveri, Lisa Oh

 Find in git repository BalkcomBots.
 '
 """

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

def compute_vel_from_angle(error):

    #If error is greater than one, keep going the same speed
    #else, slow down the robot as it approaches the target distance
    if error > 1:
        return .5
    else:
        temp_vel = error/2
    if temp_vel < .1:
        return .1
    else:
        return temp_vel

def compute_vel_from_dist(error):

    #If error is greater than one, keep going the same speed
    #else, slow down the robot as it approaches the target distance

    #Hardcoded upperbound and lowerbound
    if error > .5:
        return .15
    else:
        temp_vel = error *.3
    if temp_vel < .015:
        return .015
    else:
        return temp_vel

class Forward:
    def __init__(self, state, dist):
        self.state = state
        self.done = False
        self.target_x = self.state.x + dist * cos(self.state.angle)
        self.target_y = self.state.y + dist * sin(self.state.angle)
        self.target_point = array((self.target_x, self.target_y))
        self.total_dist = dist

    def act(self):
        #Error gives us our current distance from our target position.
        error = norm(self.target_point - array((self.state.x, self.state.y)))
        rospy.loginfo("Current x, y: " + str(self.state.x) + ", " + str(self.state.y))

        #if still not at target, compute the next move to make.
        if(error > .02):

            move_cmd = Twist()
            move_cmd.linear.x = compute_vel_from_dist(error)
            self.state.cmd_vel.publish(move_cmd)

        else:
            #Push an empty Twist to stop the robot and finish the action
            self.state.cmd_vel.publish(Twist())
            self.done = True

    def __str__(self):
        print "drove " + str(self.total_dist) + " meters forwards"

class Turn:
    def __init__(self, state, angle):
        self.state = state

        self.target_angle = angle

        provisional = self.target_angle - self.state.angle

        if provisional >= pi:
            self.turn = provisional - 2*np.pi
            
        elif provisional <= -pi:
            self.turn = provisional + 2*np.pi
        else:
            self.turn = provisional
        if self.turn > 0:
            self.direction = 'L'
        else:
            self.direction = 'R'

        self.target_angle = rectify_angle_pi(self.target_angle)
        rospy.loginfo("Target angle: " + str(self.target_angle))
        self.done = False
        self.turning_angle = angle

        print "here"
        print provisional, self.target_angle, self.state.angle, self.direction
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

class PenDraw:
    #get desired x_p, y_p,
    #comput P^-1 = [v,w]
    #Send [v,w] to robot, probably using our turn and drive functions
    #which nets us a new Quwu
    #we stop when we find it reaches a certain distance.
    def __init__(self,state, penv, draw_time):
        self.state = state
        self.penv = penv
        self.rad = .05
        self.draw_time = draw_time
        self.done = False

    def act(self):
        move_cmd = Twist()
        rospy.loginfo("Target velocity: " + str(self.penv))
        start_time = rospy.get_time()
        print ("start_time = " + str(start_time))
        while rospy.get_time() < start_time + self.draw_time:
            self.updateposition(move_cmd)
            self.state.cmd_vel.publish(move_cmd)
            # rospy.sleep(20)
            print(rospy.get_time())
        rospy.loginfo("Target velocity: " + str(self.penv))
        #when should we stop it?
        self.state.cmd_vel.publish(Twist())
        self.done = True

    def computeJac(self):
        Jac = np.zeros(shape=(2,2))
        Jac[0] = [np.cos(self.state.angle), -self.rad * np.sin(self.state.angle)]
        Jac[1] = [np.sin(self.state.angle), self.rad * np.cos(self.state.angle)]
        return Jac

    def inverseJac(self, Jac):
        print("here")
        print(str(Jac))
        return inv(Jac)

    def updateposition(self, movecmd):
        #Multiply by the timestep -- which is the rate
        #should give [v,w]
        control = np.matmul(self.inverseJac(self.computeJac()), self.penv)

        movecmd.linear.x = control[0]
        movecmd.angular.z = control[1]

class TurtlebotState:
    def __init__(self):
        # start up the subscribers to monitor state

        self.subscriber_odom = rospy.Subscriber("/odom", Odometry, self.update_odom)
        self.laser_msg = rospy.Subscriber("/scan", LaserScan, self.update_laser)
        self.x = None
        self.y = None
        self.angle = None
        self.ranges = None
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
        print len(self.ranges)
        self.anginc = scan.angle_increment
        self.anglestart = scan.angle_min 
        self.angle_max = scan.angle_max
        self.rmax = scan.range_max
        self.rmin = scan.range_min
       
        print (self.rmin in self.ranges)
        self.laserReady = True

    def update_odom(self, msg):
        self.angle = yaw_from_odom(msg)
        #print (str(msg.pose.pose.position))
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.odomReady = True

    def shutdown(self):
        rospy.loginfo("Shutting down turtlebot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
        rospy.loginfo("Goodbye.")

class LaserTurn:
    def __init__(self,state):
        self.state = state
        self.done = False
        self.rate = rospy.Rate(20)

    def act(self):
        closest = float("inf")
        index = None
        for i in xrange(len(self.state.ranges)):
            if self.state.ranges[i] < closest:
                if self.state.ranges[i] >= self.state.rmin:
                    closest = self.state.ranges[i]
                    index = i

        self.targetangle = index * self.state.anginc + self.state.angle
        self.targetangle = rectify_angle_pi(self.targetangle)
        print self.state.angle, self.state.anglestart, index, closest
        turnfunc = Turn(self.state, self.targetangle)
        while not turnfunc.done:
            turnfunc.act()
            self.rate.sleep()
        self.done = True



    def follow(self):
        print self.state.anglestart
        print self.state.angle_max

        closest = float("inf")
        index = None
        for i in xrange(45):
            if self.state.ranges[i] < closest:
                if self.state.ranges[i] >= self.state.rmin:
                    closest = self.state.ranges[i]
                    index = i

        for i in xrange(315,360):
            if self.state.ranges[i] < closest:
                if self.state.ranges[i] >= self.state.rmin:
                    closest = self.state.ranges[i]
                    index = i

        self.targetangle = index * self.state.anginc + self.state.angle
        self.targetangle = rectify_angle_pi(self.targetangle)
        print self.state.angle, self.state.anglestart, index, closest
        turnfuc = Turn(self.state, self.targetangle)
        while not turnfuc.done: 
            turnfuc.act()
            self.rate.sleep()
        # forwardfuc = Forward(self.state)
        

        estdist = self.state.ranges[index]
        drive_dist = estdist - .5

        if drive_dist > 0:
            forwardfuc = Forward(self.state, drive_dist)
            while not forwardfuc.done:
                forwardfuc.act()
                self.rate.sleep()
        self.done = True




class Ducklings:
    def __init__(self,state):
        self.state = state
        self.done = False
        self.rate = rospy.Rate(20)


       

    def act(self):
        closest = float("inf")
        index = None
        for i in xrange(45):
            if self.state.ranges[i] < closest:
                if self.state.ranges[i] >= self.state.rmin:
                    closest = self.state.ranges[i]
                    index = i

        for i in xrange(315,360):
            if self.state.ranges[i] < closest:
                if self.state.ranges[i] >= self.state.rmin:
                    closest = self.state.ranges[i]
                    index = i

        self.targetangle = index * self.state.anginc + self.state.angle
        self.targetangle = rectify_angle_pi(self.targetangle)

        #Forward
        estdist = self.state.ranges[index]
        drive_dist = estdist - .5
        move_cmd = Twist()
        if drive_dist > 0: 

            self.target_x = self.state.x + drive_dist * cos(self.state.angle)
            self.target_y = self.state.y + drive_dist * sin(self.state.angle)
            self.target_point = array((self.target_x, self.target_y))
            self.total_dist = drive_dist

            error = norm(self.target_point - array((self.state.x, self.state.y)))
            rospy.loginfo("Current x, y: " + str(self.state.x) + ", " + str(self.state.y))

            #if still not at target, compute the next move to make.
            if(error > .02):

                
                move_cmd.linear.x = compute_vel_from_dist(error)
        



        provisional = self.targetangle - self.state.angle
        if provisional != 0:
            if provisional >= pi:
                self.turn = provisional - 2*np.pi
                
            elif provisional <= -pi:
                self.turn = provisional + 2*np.pi
            else:
                self.turn = provisional
            if self.turn > 0:
                self.direction = 'L'
            else:
                self.direction = 'R'

            self.targetangle = rectify_angle_pi(self.targetangle)
            rospy.loginfo("Target angle: " + str(self.targetangle))
            self.done = False
            

        #turn act
            error = abs(self.targetangle - self.state.angle)
            rospy.loginfo("Current angle: " + str(self.state.angle))

            if (error > .02):
                if self.direction == 'L':
                    
                    move_cmd.angular.z = compute_vel_from_angle(error)
                
                elif self.direction == 'R':
                    
                    move_cmd.angular.z = -compute_vel_from_angle(error)

        self.state.cmd_vel.publish(move_cmd)

            

def main():
    #?
    rospy.init_node("turn_to")

    #initialize our turtlebot to start taking actions.

    state = TurtlebotState()
    print "The angle is" + str(state.angle)
    turn_angle = -1 * state.angle
    #Sends a shutdown message to rospy to stop execution
    rospy.on_shutdown(state.shutdown)

    rate = rospy.Rate(20)
    lt = Ducklings(state)

    while not rospy.is_shutdown():
        lt.act()

    rate.sleep()



main()
