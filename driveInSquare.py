import rospy
import sys
import math
from geometry_msgs.msg import Twist

def init():
    rospy.init_node('GoForward', anonymous=False)

def timed_drive(seconds, velocity):
    r = rospy.Rate(1)
    cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    move_cmd = Twist()
    move_cmd.linear.x = velocity
    
    # For each second publish the forward velocity to robot
    for i in range(seconds):    
        cmd_vel.publish(move_cmd)
        r.sleep()

    stop()

def timed_left(seconds, velocity):
    r = rospy.Rate(1)
    cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    move_cmd = Twist()
    move_cmd.angular.z = velocity

    # For each second publish the left angular velocity to robot
    for i in range(seconds):
        cmd_vel.publish(move_cmd)
        r.sleep()

    stop()


def timed_right(seconds, velocity):
    r = rospy.Rate(1)
    cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    move_cmd = Twist()
    move_cmd.angular.z = velocity

    # For each second publish the left angular velocity to robot
    for i in range(seconds):
        cmd_vel.publish(move_cmd)
        r.sleep()

    stop()
   

def control_turtlebot():
    a = None

    while True:
        a = raw_input("Action, Seconds")

        if a == "":
            break

        # Parse user input
        inputs_list = a.split()
        action = inputs_list[0]
        seconds = int((inputs_list[1]))

        if action == "f":
            timed_drive(seconds, .1)
        elif action == "l":
            timed_left(seconds, (math.pi/10))
        elif action == "r":
            timed_right(seconds, (-math.pi/10))
        else:
            continue
        
        stop()  
        
# def drive(meters):
#     r = rospy.Rate(1)
#     cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
#     move_cmd = Twist()
#     if meters > 0 : 
#         velocity = 0.1
#     else: 
#         velocity = -0.1
#     seconds = abs(float(meter)/velocity)
#     move_cmd.linear.x = velocity

#     for i in range(seconds):
#         cmd_vel.publish(move_cmd)
#         r.sleep()

# def turn(radians):
#     r = rospy.Rate(1)
#     cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
#     move_cmd = Twist()
#     if meters > 0 : 
#         ang_vel = 0.1
#     else: 
#         ang_vel = -0.1
#     seconds = abs(float(radians)/ang_vel)
#     move_cmd.angular.z = ang_vel

#     for i in range(seconds):
#         cmd_vel.publish(move_cmd)
#         r.sleep()

def square():
    a = None

    while True:
        a = raw_input("Square size")
        forward_velocity = 0.1
        turn_velocity = math.pi/10
        
        if a == "":
            break

        size = float(a)
        forward_seconds = int((size/forward_velocity))
        turn_seconds = int(((math.pi/2)/turn_velocity))

        for i in range(4):
            timed_drive(forward_seconds, forward_velocity)
            timed_right(turn_seconds, turn_velocity)

def main():
    
    control_turtlebot()
    square()

def stop():
    cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.loginfo("Stop turtlebot")
    cmd_vel.publish(Twist())
    rospy.sleep(1)


init()
main()
stop()
