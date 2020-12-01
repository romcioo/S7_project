#from controller import Robot
from controller import InertialUnit
from controller import Gyro, DistanceSensor, RangeFinder
from controller import Motor, Supervisor#, Node
import math
import time
import rospy
import os
# import time
import sys
import numpy as np

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int16, Bool
from std_msgs.msg import Float32, Float64MultiArray
from std_msgs.msg import String
from sensor_msgs.msg import Image

import griding

##[Functions]

##[Callback]
def headingCallback(msg):
	global heading_msg
	heading_msg = msg
	pass

def finishCallback(msg):
	global finished
	finished = msg.data
	pass
##![Callback]

## [Drive - Side]
def drive_side(msg):
	angle = msg.y
	distance = msg.x

	##[Control drive]
	side = angle/(10*math.pi/180)
	if abs(side) > 2:
		side = 2*side/abs(side)
	try:
		drive = -(2/abs(side) - 1)
	except:
		drive = 0
	if abs(drive) > 4:
		drive = -4
		
	if distance <= 1.5 and abs(drive) >= 2:
		drive = -2


	return drive, side
## ![Drive - Side]

## [Add noise]
def addNoise(real_point):
	x,y,z = real_point
	
	xz_variance = 1.02
	y_variance = xz_variance*.5
	
	x += np.random.normal(loc=0, scale=xz_variance)
	y += np.random.normal(loc=0, scale=y_variance)
	z += np.random.normal(loc=0, scale=xz_variance)
	
	return x,y,z
## ![Add noise]

def visitedPointCB(data, fileObject, tr, area, overlap, true_pos_list):
	x_coord = data.x
	y_coord = data.y
	z_coord = data.z
	time = data.w

	s = "{},{},{},{}".format(x_coord,y_coord,z_coord,time)
	s += mtrx2str(tr)

	s += "{},{},".format(area,overlap)
	s += "{},{},{}".format(true_pos_list[0], true_pos_list[1], true_pos_list[2])

	msgData = s + "\n"
	fileObject.write(msgData)

def mtrx2str(m):
	s = ","
	for row in m:
		for num in row:
			s += str(num) + ","
	return s[:-1]

def setupFile(n):

	fn = "/home/ros/Documents/my_test_project/refList/visited_point{}.txt".format(n)
	fileObject = open(fn, 'w')

	shipFile = "text"
	f = open("/home/ros/underdasea/controllers/controller_1/fileNames.txt","r")
	read = f.read()
	read = read.split("\n")
	if n <= len(read):
		shipFile = read[n-1]
	f.close()
	shipFile += "\n"
	"""with f as open("fileNames.txt","r"):
	    read = f.read()
	    read = read.split("\n")
	    if n <= len(read):
	        shipFile = read[n-1]
	    f.close()"""
	fileObject.write(shipFile)
	scaleFactor = "{},{},{},{}".format(1.0,1.0,1.0,1)
	one = ",0"
	scaleFactor += 14*one
	scaleFactor += "\n"
	fileObject.write(scaleFactor)
	serialNumber = str(n) + "\n"
	fileObject.write(serialNumber)

	return fileObject

def saveMatrix(grid, n):
	rows = len(grid)
	cols = len(grid[0])

	fn = "/home/ros/Documents/my_test_project/refList/visited_matrix{}.txt".format(n)
	matrixFile = open(fn,"w")
	matrixFile.write("{},{}\n".format(rows,cols))
	for row in grid:
		for el in row:
			matrixFile.write("{},{}\n".format(el[0],el[1]))

	matrixFile.close()

##![Functions]

##[Declare]
#global n
# create the Robot instance.
robot = Supervisor()
robot_node = robot.getSelf()
# print(robot_node)
nFile = open("/home/ros/Documents/my_test_project/refList/number.txt","r")
n = int(nFile.readline())
nFile.close()

figs = ["plane","plane","plane","plane","plane","plane","plane","plane","plane","plane","plane","plane","plane","plane","plane","plane","plane","plane"]
# figureTypeFile = open("","r")
# figureType = figureTypeFile.read()
# figureTypeFile.close()
#
# figureType.split("\n")
figureType = figs[n-1]

grid = griding.create(figureType)

# get the time step of the current world.-
#timeStep = int(robot.getBasicTimeStep())
timeStep = 5000#200
# tsp = rospy.Time()
tsp = timeStep/1000.0
# print("Time Step is: "+str(timeStep))

# Get and enable devices.
rospy.init_node('python_submarine_controller', anonymous=True) # node is called 'python_webots_controller'
# rospy.loginfo("Loading Webots Controller")

##[Publishers]
imu_pub = rospy.Publisher('imuValues', Vector3, queue_size=1000)
speed_pub = rospy.Publisher("headingSpeed",Vector3,queue_size=10)
velocity_pub = rospy.Publisher('submarineVelocity', Float32, queue_size=10)
path_pub = rospy.Publisher("path", Quaternion, queue_size=1000)
camera_pub = rospy.Publisher('python_submarine_camera_images', Image, queue_size=10)
rearcamera_pub = rospy.Publisher('python_submarine_rear_camera_images', Image, queue_size=10)
true_pos_pub = rospy.Publisher('true_position', Quaternion, queue_size=1000)
reset_pub = rospy.Publisher("reset",Bool,queue_size=2)
row1_pub = rospy.Publisher("row1",Vector3,queue_size=2)
row2_pub = rospy.Publisher("row2",Vector3,queue_size=2)
row3_pub = rospy.Publisher("row3",Vector3,queue_size=2)
rows = [row1_pub, row2_pub, row3_pub]
ravin_pub = rospy.Publisher("ravin",Bool,queue_size=1)
#n_pub = rospy.Publisher("number",Int16,queue_size=1)
##![Publishers]

##[Subscriber]
rospy.Subscriber("heading",Vector3,headingCallback)
rospy.Subscriber("finish",Bool,finishCallback)
#rospy.Subscriber("number",Int16,numberCallback)
##![Subscriber]

reset_pub.publish(Bool(True))

##[Sensors]
IMUsensor = robot.getInertialUnit('inertial unit')  # front central proximity sensor
IMUsensor.enable(timeStep)

GPSsensor = robot.getGPS('gps')
GPSsensor.enable(timeStep)

GYROsensor = robot.getGyro("gyro")
GYROsensor.enable(timeStep)

# ravin_RL = robot.getDistanceSensor('Rear_Left_Edge_Det')
# print(ravin_RL)
# ravin_RL.enable(timeStep)
# ravin_FL = robot.getDistanceSensor('Front_Left_Edge_Det')
# ravin_FL.enable(timeStep)
# ravin_FR = robot.getDistanceSensor('Front_Right_Edge_Det')
# ravin_FR.enable(timeStep)
# ravin_RR = robot.getDistanceSensor('Rear_Right_Edge_Det')
# ravin_RR.enable(timeStep)
##[Sensors]
#ravins = []
#ravNames = ['Rear_Left_Edge_Det', 'Front_Left_Edge_Det', 'Front_Right_Edge_Det', 'Rear_Right_Edge_Det']
#for i in range(4):
#	ravins.append(robot.getDistanceSensor(ravNames[i]))
#	ravins[i].enable(timeStep)

##[set robot initial values]
front_left_motor = robot.getMotor("front left thruster")
front_right_motor = robot.getMotor("front right thruster")
rear_left_motor = robot.getMotor("rear left thruster")
rear_right_motor = robot.getMotor("rear right thruster")
front_left_motor.setPosition(float('inf'))
front_right_motor.setPosition(float('inf'))
rear_left_motor.setPosition(float('inf'))
rear_right_motor.setPosition(float('inf'))
front_left_motor.setVelocity(0.0)
front_right_motor.setVelocity(0.0)
rear_left_motor.setVelocity(0.0)
rear_right_motor.setVelocity(0.0)

FL_wheel = robot.getMotor("front left wheel")
FR_wheel = robot.getMotor("front right wheel")
RL_wheel = robot.getMotor("rear left wheel")
RR_wheel = robot.getMotor("rear right wheel")
FL_wheel.setPosition(float('inf'))
FR_wheel.setPosition(float('inf'))
RL_wheel.setPosition(float('inf'))
RR_wheel.setPosition(float('inf'))
FL_wheel.setVelocity(0.0)
FR_wheel.setVelocity(0.0)
RL_wheel.setVelocity(0.0)
RR_wheel.setVelocity(0.0)
##![set robot initial values]

robot.step(timeStep)
xpos, altitude , zpos = GPSsensor.getValues()

time = 0
true_pos = Quaternion(xpos, altitude, zpos, time)
robot_pose = (xpos, altitude, zpos)
xpos_old = xpos
altitude_old = altitude
zpos_old = zpos

## [Initialize]
heading_msg = Vector3(0,0,0)
finished = False

x_bound = [-48.7, 55.7]
y_bound = [-16, 28.2]
area = 0
overlap = 0
# robot_pose = (0,0,0)
pose = Quaternion()

thrust = 10000.0/1.5

fileObject = setupFile(n)

# ravinThreshold =
## ![Initialize]

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timeStep) != -1:
	# Read the sensors:
	roll, pitch, heading = IMUsensor.getRollPitchYaw()
	orient = np.array(robot_node.getOrientation())
	orient = orient.reshape(3,3)
	tr = np.transpose(orient)

	for i in range(len(rows)):
		line = tr[i]
		thing = Vector3(line[0], line[1], line[2])
		rows[i].publish(thing)

	#tr = tr.reshape(1,9)[0]
	#tr_pub = Float64MultiArray()
	#tr_pub.data = tr
	#trans_pub.publish(tr_pub)
	#robot_node = robot.getFromDef("Submarine Robot")
	#print(robot_node)
	"""if roll < 0:
	    roll = math.pi - roll
	if pitch < 0:
	    pitch = math.pi/2 - pitch
	if heading < 0:
	    heading = math.pi - heading"""

	# ravin_pub.publish(ravins[1].getMaxValue())
	# res = []
	# for el in ravins:
	# 	res.append(el.getValue())
	# rospy.loginfo(res)

	#pitch += math.pi/2 # maybe
	xpos, altitude , zpos = GPSsensor.getValues()
	roll_vel, bleh, pitch_vel = GYROsensor.getValues()
	time = robot.getTime()

	## [Publish true robot position]
	true_pos_list = [xpos, altitude , zpos]
	true_pos = Quaternion(xpos, altitude , zpos, time)
	true_pos_pub.publish(true_pos)
	## ![Publish true robot position]

	## [Publish sensors position]
	robot_pose = addNoise((xpos, altitude , zpos))
	pose = Quaternion(robot_pose[0], robot_pose[1], robot_pose[2], time)
	path_pub.publish(pose)
	thisArea, thisOverlap, grid = griding.visit(robot_pose, grid, figureType)
	area += thisArea
	overlap += thisOverlap
	visitedPointCB(pose, fileObject, tr, area, overlap, true_pos_list)

	edge = False
	if xpos >= x_bound[1] or xpos <= x_bound[0]:
		edge = True
	if altitude >= y_bound[1] or altitude <= y_bound[0]:
		edge = True
	ravin_pub.publish(Bool(edge))
	## ![Publish sensors position]

	##[Derivate position for speed]
	xSpeed = (xpos-xpos_old)/tsp
	ySpeed = (altitude-altitude_old)/tsp
	zSpeed = (zpos-zpos_old)/tsp
	##[Derivate position for speed]

	##[Update position]
	xpos_old=xpos
	altitude_old=altitude
	zpos_old=zpos
	##[Update position]

	fl_wheel=0
	fr_wheel=0
	rl_wheel=0
	rr_wheel=0

	radcoeff = 1#180.0/math.pi
	scaling = 1

	imu_pub.publish(Vector3((roll)*radcoeff*scaling, (pitch)*radcoeff*scaling, (heading)*radcoeff*scaling))
	speed_pub.publish(Vector3(math.cos(heading)*xSpeed*-1+math.sin(heading)*zSpeed*-1,ySpeed,math.sin(heading)*xSpeed+math.cos(heading)*zSpeed))

	velocity_pub.publish(bleh)

	# if logger==True:
	#     f.write(str(xpos)+","+str(altitude)+","+str(zpos)+"\n")
	"""
	Here it goes all the modes
	"""
	##[Control drive] side = 1 is counter-clockwise
	drive, side = drive_side(heading_msg)
	#rospy.loginfo(side)
	#drive = 0
	#side = 1
	##![Control drive]
	k_d = 1
	k_s = 1

	fl_wheel=k_d*drive+k_s*side
	fr_wheel=k_d*drive-k_s*side
	rl_wheel=k_d*drive+k_s*side
	rr_wheel=k_d*drive-k_s*side

	front_left_motor.setVelocity(-thrust)#positive is up  #0.44467908653
	front_right_motor.setVelocity(thrust)#negative is up #0.44616503673
	rear_left_motor.setVelocity(thrust)#negative is up     #0.42589835641
	rear_right_motor.setVelocity(-thrust)#positive is up  #0.42744959936

	FL_wheel.setVelocity(fl_wheel)
	FR_wheel.setVelocity(fr_wheel)
	RL_wheel.setVelocity(rl_wheel)
	RR_wheel.setVelocity(rr_wheel)

	if finished:
		#rospy.loginfo("in")
		bool_pub = Bool()
		time = 0
		fileObject.close()

		saveMatrix(grid,n)

		n += 1
		#fileObject = setupFile(n)
		#n_pubs = Int16()
		#n_pubs.data = n
		#n_pub.publish(n_pubs)
		nFile = open("/home/ros/Documents/my_test_project/refList/number.txt","w")
		nFile.write(str(n))
		nFile.close()
		robot.simulationQuit(1)
		#sys.exit(0)
	pass
