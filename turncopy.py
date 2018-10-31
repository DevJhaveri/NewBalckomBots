"""Turn function for CS81 HW3.
Team Members: Andrew Sosanya, Anne Sherrill, Dev Jhaveri, Lisa Oh

 Find in git repository BalkcomBots.
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
from numpy import invert as inv


from angles import rectify_angle_pi

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
        print "drove " + dist + " meters forwards"


class TurnLeft:
    def __init__(self, state, angle):
        self.state = state
        self.target_angle = rectify_angle_pi(state.angle + angle)
        rospy.loginfo("Target angle: " + str( self.target_angle))
        self.done = False

    def act(self):
        #if still not at target, compute the next move to make.

        error = abs(self.target_angle - self.state.angle)
        rospy.loginfo("Current angle: " + str( self.state.angle))

        if(error > .02):
            move_cmd = Twist()
            move_cmd.angular.z = compute_vel_from_angle(error)
            self.state.cmd_vel.publish(move_cmd)

        else:
            self.state.cmd_vel.publish(Twist())
            self.done = True

    def __str__(self):
        print "Turned Left by " + angle + " radians"
class TurnRight:
    def __init__(self, state, angle):
        self.state = state
        self.target_angle = rectify_angle_pi(state.angle - angle)
        print ("Target angle is" + str(state.angle))
        rospy.loginfo("Target angle: " + str( self.target_angle))
        self.done = False

    def act(self):
        error = abs(self.target_angle - self.state.angle)
        rospy.loginfo("Current angle: " + str( self.state.angle))

        if(error > .02):
            move_cmd = Twist()
            move_cmd.angular.z = -compute_vel_from_angle(error)
            self.state.cmd_vel.publish(move_cmd)

        else:
            self.state.cmd_vel.publish(Twist())
            self.done = True

    def __str__(self):
        print "Turned right by " + angle + " radians"

class PenDraw:
    #get desired x_p, y_p,
    #comput P^-1 = [v,w]
    #Send [v,w] to robot, probably using our turn and drive functions
    #which nets us a new Quwu
    #we stop when we find it reaches a certain distance.
    def __init__(self,state,penv):
        self.state = state
        self.penv = penv
        self.rad = .5


    def act(self, draw_time):
        move_cmd = Twist()
        rospy.loginfo("Target velocity: " + str(self.penv))
        for i in xrange(draw_time):
            updateposition(move_cmd)
            self.state.cmd_vel.publish(move_cmd)
            rospy.sleep(1)
        rospy.loginfo("Target velocity: " + str(self.penv))

        #when should we stop it?


    def computeJac(self):

        Jac = np.zeros(shape=(2,2))
        Jac[0] = [np.cos(state.angle), -self.rad * np.sin(state.angle)]
        Jac[1] = [np.sin(state.angle), self.rad * np.cos(state.angle)]
        return Jac

    def inverseJac(self, Jac):
        return inv(Jac)

    def updateposition(self, movecmd):
        #Multiply by the timestep -- which is the rate
        #should give [v,w]
        control = inverseJac(self.computeJac) * self.penv

        movecmd.linear.x = control[0]
        movecmd.angular.z = control[1]

class TurtlebotState:
    def __init__(self):
        # start up the subscribers to monitor state

        self.subscriber_odom = rospy.Subscriber("/odom", Odometry, self.update_odom)
        self.x = None
        self.y = None
        self.angle = None
        #ready indicates whether the odometry is received.
        self.ready = False

        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


        # wait until odometry received, etc before
        # returning control
        while not self.ready:
            rate = rospy.Rate(20)
            rate.sleep()

    def update_odom(self, msg):
        self.angle = yaw_from_odom(msg)
        print (str(msg.pose.pose.position))
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.ready = True

    def shutdown(self):

        rospy.loginfo("Shutting down turtlebot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
        rospy.loginfo("Goodbye.")




def main():
    #?
    rospy.init_node("turn_to")

    #initialize our turtlebot to start taking actions.

    state = TurtlebotState()
    #Sends a shutdown message to rospy to stop execution
    rospy.on_shutdown(state.shutdown)

    rate = rospy.Rate(20)

    #Queue of actions to take
    actions = [('L', pi/2), ('R', pi/2), ('F', .5)]
    #while rospy is executing method run the queue of commands.
    while not rospy.is_shutdown():
        for i in range(0, len(actions)):

            if actions[i][0] == 'L':
                current_action = TurnLeft(state, actions[i][1])
            elif actions[i][0] == 'R':
                current_action = TurnRight(state, actions[i][1])
            else:
                current_action = Forward(state, actions[i][1])

            while not current_action.done:
                current_action.act()
                rate.sleep()
            print current_action
            print state.angle
            print state.x, state.y
        break
    rate.sleep()



main()
