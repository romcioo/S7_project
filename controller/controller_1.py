from controller import Robot
from controller import InertialUnit
from controller import Gyro
from controller import Motor
import math
import time
import rospy
import os

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import String
from sensor_msgs.msg import Image

##[Functions]

##[Callback]
def headingCallback(msg):
    global heading_msg
    heading_msg = msg
    pass
##![Callback]

def side_sign(angle, front):
    if angle < 0: # if the angle is negative
        side = -1 # turn right
    else:
        side = 1

    if front: # if it's going backwards
        side = -side # change orientation

    return side

## [Drive - Side]
def drive_side(msg): # return the drive and side parameters
    angle = msg.y # get the heading angle
    distance = msg.x # get the distance to the goal

    ##[Control drive]
    if abs(angle) <= math.pi/2: # ahead of the robot
        if abs(angle) > 5*math.pi/180: # 5º
            side = side_sign(angle,True)

            drive = 0
            if distance > 5 and abs(angle) < math.pi/4: # 45º
                drive = 1
            elif distance > 2 and abs(angle) < 2*math.pi/18: # 20º
                drive = 1
        else:
            drive = 1
            if distance < 1:
                side = side_sign(angle,True)
            else:
                side = 0
    else:
        if abs(angle) < (math.pi - 5*math.pi/180): # 5º
            side = side_sign(angle,False)

            drive = 0
            if distance > 5 and abs(angle) > (math.pi - math.pi/4): # 45º
                drive = -1
            elif distance > 2 and abs(angle) > (math.pi - 2*math.pi/18): # 20º
                drive = -1
        else:
            drive = -1
            if distance < 1:
                side = side_sign(angle,False)
            else:
                side = 0
    ##![Control drive]

    return drive, side
## ![Drive - Side]

## [Add noise]
def addNoise(real_point, robot_pose):
    return real_point
## ![Add noise]

##![Functions]

##[Declare]
# create the Robot instance.
robot = Robot()

initials = [0, 0, 0] # initial orientation

# get the time step of the current world.
timeStep = int(robot.getBasicTimeStep()) # get the time step in milliseconds
tsp = timeStep/1000.0 # time step in seconds
thrust = 10000/2

# Get and enable devices.
rospy.init_node('python_submarine_controller', anonymous=True) # node is called 'python_webots_controller'

##[Publishers]
imu_pub = rospy.Publisher('imuValues', Vector3, queue_size=1000)
speed_pub = rospy.Publisher("headingSpeed",Vector3,queue_size=10)
velocity_pub = rospy.Publisher('submarineVelocity', Float32, queue_size=10)
path_pub = rospy.Publisher("path", Quaternion, queue_size=1000)
camera_pub = rospy.Publisher('python_submarine_camera_images', Image, queue_size=10)
rearcamera_pub = rospy.Publisher('python_submarine_rear_camera_images', Image, queue_size=10)
true_pos_pub = rospy.Publisher('true_position', Quaternion, queue_size=10)
##![Publishers]

##[Subscriber]
heading_sub = rospy.Subscriber("heading",Vector3,headingCallback)
##![Subscriber]

##[Sensors]
IMUsensor = robot.getInertialUnit('inertial unit')  # front central proximity sensor
IMUsensor.enable(timeStep)

GPSsensor = robot.getGPS('gps')
GPSsensor.enable(timeStep)

GYROsensor = robot.getGyro("gyro")
GYROsensor.enable(timeStep)

camera = robot.getCamera("camera")
camera.enable(timeStep)

rearcamera = robot.getCamera("rearcamera")
rearcamera.enable(timeStep)
##[Sensors]

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
time = 0 # initial time
true_pos = Quaternion(xpos, altitude, zpos, time)
robot_pose = (xpos, altitude, zpos)
xpos_old = xpos
altitude_old = altitude
zpos_old = zpos

##![Declare]

## [Initialize]
heading_msg = Vector3(0,0,0)

pose = Quaternion()

radcoeff = 180.0/math.pi
scaling = -1
## ![Initialize]

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timeStep) != -1:
    # Read the sensors:
    roll, pitch, heading = IMUsensor.getRollPitchYaw()
    xpos, altitude , zpos = GPSsensor.getValues()
    roll_vel, bleh, pitch_vel = GYROsensor.getValues()
    time += tsp # time on this iteration

    ## [Publish true robot position]
    true_pos = Quaternion(xpos, altitude , zpos, time)
    true_pos_pub.publish(true_pos)
    ## ![Publish true robot position]

    ## [Publish sensors position]
    robot_pose = addNoise((xpos, altitude , zpos), robot_pose)
    pose = Quaternion(robot_pose[0], robot_pose[1], robot_pose[2], time)
    path_pub.publish(pose)
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

    ## [Send the camera images to ros]
    ## Now we send some things to ros BELOW
    camera_image_msg = Image()
    camera_image_msg.width = 320
    camera_image_msg.height = 240
    camera_image_msg.encoding = "bgra8"
    camera_image_msg.is_bigendian = 1
    camera_image_msg.step = 1280
    camera_image_msg.data = camera.getImage()
    camera_pub.publish(camera_image_msg)


    ## Now we send some things to ros BELOW
    rearcamera_image_msg = Image()
    rearcamera_image_msg.width = 320
    rearcamera_image_msg.height = 240
    rearcamera_image_msg.encoding = "bgra8"
    rearcamera_image_msg.is_bigendian = 1
    rearcamera_image_msg.step = 1280
    rearcamera_image_msg.data = rearcamera.getImage()
    rearcamera_pub.publish(rearcamera_image_msg)
    ## ![Send the camera images to ros]

    imu_pub.publish(Vector3((roll+initials[0])*radcoeff*scaling, (pitch+initials[1])*radcoeff*scaling, (heading+initials[2])*radcoeff*scaling))
    speed_pub.publish(Vector3(math.cos(heading)*xSpeed*-1+math.sin(heading)*zSpeed*-1,ySpeed,math.sin(heading)*xSpeed+math.cos(heading)*zSpeed))

    velocity_pub.publish(bleh)

    ##[Control drive]
    drive, side = drive_side(heading_msg)
    ##![Control drive]

    fl_wheel=4*drive+4*side
    fr_wheel=4*drive-4*side
    rl_wheel=4*drive+4*side
    rr_wheel=4*drive-4*side

    front_left_motor.setVelocity(-thrust)#positive is up  #0.44467908653
    front_right_motor.setVelocity(thrust)#negative is up #0.44616503673
    rear_left_motor.setVelocity(thrust)#negative is up     #0.42589835641
    rear_right_motor.setVelocity(-thrust)#positive is up  #0.42744959936

    FL_wheel.setVelocity(fl_wheel)
    FR_wheel.setVelocity(fr_wheel)
    RL_wheel.setVelocity(rl_wheel)
    RR_wheel.setVelocity(rr_wheel)

    pass
