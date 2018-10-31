from cs1lib import *
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import pinv

FRAMERATE = 30
WIDTH = 400
HEIGHT = 400
TIMESTEP = float(1.0/FRAMERATE)
PENLENGTH = 20
PENVELOCITY = 2


# def center_vel(self.Jac, pen_vel)

class Robot:
    def __init__(self, x=0, y=0 , theta=np.pi/2, x_p=0, y_p=0, wheel_rad = 0.3, robdiam = 10):
        self.x = x
        self.y = y
        self.theta = theta
        self.omega = 0
        self.x_p = self.x + PENLENGTH*np.cos(self.theta)
        self.y_p = self.y + PENLENGTH*np.sin(self.theta)
        self.wheel_rad = wheel_rad
        self.wheel_vel = 0
        self.robdiam = robdiam
        self.vx_p = 0
        self.vy_p = 0

        self.inked = []


        #Set up variables for the Jacobian
        dxpdx = 1
        dxpdy = 0
        dxpd0= -np.sin(self.theta)
        dypdx = 0
        dypdy = 1
        dypd0= np.cos(self.theta)
        self.Jac = np.zeros(shape=(2,3))
        self.Jac[0] = [dxpdx,dxpdy,dxpd0]
        self.Jac[1] = [dypdx,dypdy,dypd0]

    def rob_angv(self, wheel_vec, wheel_rad, bot_diam):
            return wheel_rad * (wheel_vec[0] - wheel_vec[1]) / (2* bot_ddiam)

    def pen_vel(self, x_p, y_p):
        return np.array([[x_p],[y_p]])

    def update_position(self, qdot):

        self.x += qdot[0] *TIMESTEP
        self.y += qdot[1] * TIMESTEP
        #Takes care of the angular velocity??
        self.theta += qdot[2] * TIMESTEP
        # self.x_p += self.vx_p
        # self.y_p += self.vy_p
        self.x_p +=  self.vx_p * TIMESTEP
        self.y_p += self.vy_p * TIMESTEP
        # if self.x - self.robdiam < 0 or self.x + self.robdiam > WIDTH or self.y + self.robdiam > HEIGHT or self.y - self.robdiam < 0:
        #     # self.x -= 2 * qdot[0]
        #     # self.y -= 2 * qdot[1]
        #     print self.x, self.y

        #     self.vx_p = -2 * self.vx_p
        #     self.vy_p = -2 * self.v
        #     print self.x_p, self.vx_p
        #     print self.y_p, self.vy_p


    def ink(self):
        enable_stroke()
        for point in self.inked:
             draw_point(point[0], point[1])
    def draw_robot(self):
        clear()
        #draw pen

        #draw robot as pointmass
        #Set gradient
        dxpdx = 1
        dxpdy = 0
        dxpd0= -np.sin(self.theta)
        dypdx = 0
        dypdy = 1
        dypd0= np.cos(self.theta)
        #Set columns of the Jacobian.
        self.Jac = np.zeros(shape=(2,3))
        self.Jac[0] = [dxpdx,dxpdy,dxpd0]
        self.Jac[1] = [dypdx,dypdy,dypd0]
        #inverse Jacobian using Moore-Penrose inverse
        inv_Jac = pinv(self.Jac)
        #Take the dot product of the inverse Jacobian and pen velocity to get
        #The velocity vector.
        qdot = np.dot(inv_Jac,self.pen_vel(self.vx_p,self.vy_p))

        #Update the position vector with qdot, the velocity vector
        # qdot = [vx, vy, w]
        self.update_position(qdot)

        #Inverse matrix to get the wheel velocities
        jprime = np.zeros(shape=(3,2))
        jprime[0] = [np.sin(self.theta), np.sin(self.theta)]
        jprime[1] = [np.cos(self.theta), np.cos(self.theta)]
        jprime[2] = [self.robdiam, self.robdiam]
        inv_jprime = pinv(jprime)

        #velocity vector for each of the wheels.
        wheel_vec = (2/self.wheel_rad) *  np.dot(inv_jprime, qdot)


        #Interactive control directions, choose where the pen goes.
        if is_key_pressed("a"):
            self.vx_p = -PENVELOCITY
        if is_key_pressed("d"):
            self.vx_p = PENVELOCITY
        if is_key_pressed("w"):
            self.vy_p = -PENVELOCITY
        if is_key_pressed("x"):
            self.vy_p = PENVELOCITY
        if is_key_pressed("s"):
            self.vy_p = 0
            self.vx_p = 0
        #def
        # if is_key_pressed("a") :
        #     self.v_x = -0.2
        # elif is_key_pressed("b"):
        #     self.x = 0.2
        # elif is_key_pressed("w"):
        #     self.y = 0.2
        # elif is_key_pressed("x"):
        #     self.y = -0.2
        # elif is_key_pressed("s"):
        #     self.x = 0
        #     self.y = 0

        #actually drawing the robot and pen.
        self.update_position(qdot)
        enable_smoothing()
        enable_stroke()
        self.inked.append((self.x_p,self.y_p))
        set_stroke_color(0, 0, 0)
        self.ink()
        #Draw line to pen
        set_stroke_color(1, 1, 1)

        draw_line(self.x, self.y,  self.x_p, self.y_p)
        set_fill_color(1, 0, 0)
        draw_circle(self.x_p, self.y_p, 8)
        set_fill_color(0, 0, 0)
        #draw bot
        draw_circle(self.x,self.y, 12)
        enable_fill()
        set_fill_color(0,0.1, 1)
        draw_circle(self.x - self.robdiam, self.y - self.robdiam, self.robdiam/2)
        draw_circle(self.x + self.robdiam, self.y +  self.robdiam, self.robdiam/2)
        set_fill_color(0, 0, 1)
        draw_line(self.x, self.y,  self.x_p, self.y_p)

Robot = Robot(x=WIDTH/2, y=HEIGHT/2)
start_graphics(Robot.draw_robot, width=WIDTH, height=HEIGHT, framerate=60)