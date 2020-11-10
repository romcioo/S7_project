#!/usr/bin/env python3
"""Globla planner."""

import pickle as pic
import math
import time
import sys
import os
import random
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Vector3Stamped


edge = False
visitedBool = False
vistedPoint = Vector3( 0, 0, 0)
refListPath = "/home/woombat/Documents/my_test_project/refList/"
fileName = "visited_point.csv"
radii = 10
degrees = 33
rngTime =0.1

def vistedEgde(data):
    global edge
    edge = data.data
    rospy.loginfo(rospy.get_caller_id() + "We have seen an edge %s", data.data)
    rospy.spin()

def vistedCB(data):
    global visitedBool
    visitedBool = data.data
    rospy.loginfo(rospy.get_caller_id() + "We visited the point %s", data.data)
    rospy.spin()

def vistedPointCB(data):
    global refListPath, fileName
    print("New point visted")
    filename = refListPath + fileName

    fileObject = open(filename, 'w')
    time = data.header.stamp.nsecs
    x_coord = data.vector.x
    y_coord = data.vector.y
    z_coord = data.vector.z
    msgData ="time: {} x: {} y: {} z: {}".format(time,x_coord,y_coord,z_coord)
    print(msgData)
    pic.dump(msgData, fileObject)
    fileObject.close()
    rospy.spin()

def setupFile():
    #model file
    #scale factor x,y,z,bool
    #serial number structured
    global refListPath, fileName
    print("New file made")
    filename = refListPath + fileName
    fileObject = open(filename, 'w')
    shipFile = "text"
    pic.dump(shipFile, fileObject)
    scaleFactor = "{},{},{},{}\n".format(1.0,1.0,1.0,1)
    pic.dump(scaleFactor, fileObject)
    serialNumber = "serial number\n"
    pic.dump(serialNumber, fileObject)
    fileObject.close()
    rospy.spin()


def pol2cart(theta, r):
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return(x, y)

#fixed angle
def rndWalk():
    global edge, radii, degrees
    turnTheta
    if edge == False:
        x,y = pol2cart(turnTheta,radii)
    else:
        degrees = degrees + 33
        turnTheta = degrees * math.pi /180
        x,y = pol2cart(turnTheta,radii)
        edge = False
    return x,y

#rng Turn
def rndWalkRngTurn():
    global theta, edge, rngTime
    radii = 10
    degrees
    turnTheta
    x,y
    now = time.time()
    elapsed = now - rngTime
    if edge == True and elapsed >  60:
        degrees = degrees + random.randint(1,40)
        turnTheta = degrees * math.pi /180
        x,y = pol2cart(turnTheta,radii)
        edge = False
        rngTime = time.time()
    else:
        x,y = pol2cart(turnTheta,radii)
    return x,y


def mainGlobalPlan():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('globalPlanner', anonymous=True)
    sleep = rospy.Rate(5)
    pub_goal = rospy.Publisher("goal", Vector3, queue_size=1)
    rospy.Subscriber("reachPoint", Bool, vistedCB)
    rospy.Subscriber("reachEdge", Bool, vistedEgde)
    rospy.Subscriber("path",Vector3Stamped,vistedPointCB)

    setupFile()
    while not rospy.is_shutdown():
        global visitedBool
        #follow ref. list
        data = False
        navigationFunc = False
        if data == True:
            csv_file = refListPath + "refList1.csv"
            with open(csv_file, 'r') as csv_file:
                for line in csv_file:
                    line = line.strip('\n')
                    #print(line)
                    coord = line.split(' ') # take the pickled obj
                    x_coord = coord[0]
                    y_coord = coord[1]
                    z_coord = coord[2]
                    #print("x coord: {}   y coord: {}    z z_coord: {}").format(x_coord,y_coord,z_coord)
                    #to localPlanner returns points that has been missing
                    while visitedBool == False:
                        goalPoint = Vector3()
                        goalPoint.x = float(x_coord)
                        goalPoint.y = float(y_coord)
                        goalPoint.z = float(z_coord)
                        pub_goal.publish(goalPoint)
                        sleep.sleep()
                    visitedBool = False


        #Generate path
        if navigationFunc == True:
            ranWalk = False
            rngWalk = False
            if rngWalk == True:
                x,y = rndWalkRngTurn()
                goalPoint = Vector3()
                goalPoint.x = float(x)
                goalPoint.y = float(y)
                goalPoint.z = float(0)
                pub_goal.publish(goalPoint)
                egde = False
                sleep.sleep()
            if ranWalk == True:
                x,y = rndWalk()
                goalPoint = Vector3()
                goalPoint.x = float(x)
                goalPoint.y = float(y)
                goalPoint.z = float(0)
                pub_goal.publish(goalPoint)
                egde = False
                sleep.sleep()



        #close down webots
        nomoves = False
        if nomoves == True:
            print("Quiting")
            rospy.spin()
            os.kill(0,True)


        pass

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        mainGlobalPlan()
    except rospy.ROSInterruptException:
        pass
