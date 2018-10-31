import matplotlib.pyplot as plt
from math import cos
from math import sin
from math import floor
from math import radians
import numpy as np


def make_coor_array():
    with open('Coordinates.txt', 'r') as coordinates:
        coor_array = [tuple(map(float, line.split())) for line in coordinates]
    return coor_array

def plot_coor_array(coordinate_array):
    plt.scatter(*zip(*coordinate_array), marker=',', linewidths=.01)
    plt.scatter(x=0, y=0, marker='o')
    plt.show()

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
plot_coor_array(coor_array)
array = make_Hough_array()
hough = fill_Hough_array(array, coor_array)
plt.pcolormesh(hough)
plt.show()

r_list = []
theta_list = []
for theta in xrange(0,360):
    for r in xrange(0,45):
        if hough[theta][r] >= 110:
            r_list.append(r/9.0)
            theta_list.append(theta)
print r_list
print theta_list

def graph(formula, x_range):
    x = np.array(x_range)
    y = eval(formula)
    plt.plot(x, y)
    plt.show()

def formula(x, x_0, y_0):
    if abs(y_0) <= .00001:
        slope = 9999999
    else:
        slope = -x_0 / y_0
    return(slope*(x-x_0)+y_0)

def calc_lines(r_list, theta_list, coordinate_array):
    num_lines = len(r_list)
    plt.axis([-5, 5, -5, 5])
    for i in xrange(0, num_lines):
        x_0 = r_list[i]*cos(radians(theta_list[i]))
        y_0 = r_list[i]*sin(radians(theta_list[i]))
        range = xrange(-5, 5)
        plot_line(formula, range, x_0, y_0)
    plot_coor_array(coordinate_array)
    plt.show()


def plot_line(formula, x_range, x_0, y_0):
    x = np.array(x_range)
    y = formula(x, x_0, y_0)
    plt.plot(x, y)

calc_lines(r_list, theta_list, coor_array)