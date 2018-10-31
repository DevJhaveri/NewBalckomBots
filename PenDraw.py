import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import numpy as np
from numpy.linalg import pinv
from math import pi

from angles import rectify_angle_pi





class PenDraw:
    def __init__(self,state):

        self.state = state
    def act(self,pvx, pvy):
        move_cmd = Twist()
        updateposition(move_cmd)
        self.state.cmd_vel.publish(move_cmd)
        rospy.loginfo("Target velocity: " + str( self.target_angle))
        self.done = False


    def computeJac(self):

    def inverseJac(self):
        jprime = np.zeros(shape=(3,2))
        jprime[0] = [np.sin(self.theta), np.sin(self.theta)]
        jprime[1] = [np.cos(self.theta), np.cos(self.theta)]
        jprime[2] = [self.robdiam, self.robdiam]
        inv_jprime = pinv(jprime)
        return inv_jprime

    def updateposition(self,q, movecmd):
        movecmd.linear.x += qdot[0]
        movecmd.linear.y += qdot[1]
        movecmd.angular.z += qdot[2]

