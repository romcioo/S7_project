#!/usr/bin/env python
#from controller import Robot
#from controller import InertialUnit
#from controller import Gyro
#from controller import Keyboard
#from controller import Motor
import controller
import rospy
import math
import time
import os

"""from geometry_msgs.msg import Vector3
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import String"""
import geometry_msgs
import std_msgs

def main():
    # initialize ros
    rospy.init_node("planner", anonymous=True)

    # Publishers and Subscriber
    heading_pub = rospy.Publisher("heading", geometry_msgs.msg.Quaternion, queue_size=1000)
    visited_pub = rospy.Publisher("path", geometry_msgs.msg.Vector3, queue_size=1000)
    reached_pub = rospy.Publisher("reachPoint", bool, queue_size=1000)
    position_shifted_pub = rospy.Publisher("robot_pose_shifted", geometry_msgs.msg.Vector3, queue_size=1000)

    objective_sub = rospy.Subscriber("goal", geometry_msgs.msg.Vector3, objectiveCallback)
    global_position_sub = rospy.Subscriber("robot_pose", geometry_msgs.msg.Vector3, globalCallback)

    r = rospy.loop_rate(10)

    while not rospy.is_shutdown():


        r.sleep()

def objectiveCallback(msg):
    return

def globalCallback(msg):
    return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
