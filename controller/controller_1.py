from controller import Robot
from controller import InertialUnit
from controller import Gyro
from controller import Motor
import math
import time
import rospy
import os
# import time

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

def clamp(value, low, high):
    if value < low:
        return low
    elif value > high:
        return high
    return value

def side_sign(angle, front):
    if angle < 0:
        side = -1
    else:
        side = 1

    if front:
        side = -side

    return side

## [Drive - Side]
def drive_side(msg):
    angle = msg.y
    distance = msg.x

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

pitch_disturbance = 0
roll_disturbance = 0
yaw_disturbance = 0

initials = [0, 0, 0]

# get the time step of the current world.
timeStep = int(robot.getBasicTimeStep())
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

##[Controller constants]
k_roll_p = 600         # P constant of the roll PID.
k_pitch_p = 600       # P constant of the pitch PID.
k_yaw_p = 500
k_roll_d = 800
k_pitch_d = 800
k_yaw_d = 300

target_altitude = 9.0
k_vertical_thrust = 398.3476#187.28#592.2569#398.3476#327.3495#128.1189 # with this thrust, the drone lifts.
#k_vertical_offset = 0.1   # Vertical offset where the robot actually targets to stabilize itself.
k_vertical_p = 300       # P constant of the vertical PID.
k_vertical_d = 1000
##![Controller constants]

robot.step(timeStep)
xpos, altitude , zpos = GPSsensor.getValues()
# true_pos_vector = Vector3(xpos, altitude, zpos)
# true_pos = Quaternion()
# true_pos.vector = true_pos_vector
time = 0
# true_pos.header.stamp.secs = time
true_pos = Quaternion(xpos, altitude, zpos, time)
robot_pose = (xpos, altitude, zpos)
# true_pos_pub.publish(true_pos)
xpos_old = xpos
altitude_old = altitude
zpos_old = zpos

"""# drone_mode = True
# atache_mode = False
# car_mode = False
# follow_mode = False"""
altitude_bool = False
angle_dist = 0.785398
angle_lock = -2.0944

clampval = 10000

camera = robot.getCamera("camera")
camera.enable(timeStep)

rearcamera = robot.getCamera("rearcamera")
rearcamera.enable(timeStep)

lock_on = 10
front_left_motor_input = lock_on*k_vertical_thrust
front_right_motor_input = lock_on*k_vertical_thrust
rear_left_motor_input = lock_on*k_vertical_thrust
rear_right_motor_input = lock_on*k_vertical_thrust
##![Declare]

## [Initialize]
heading_msg = Vector3(0,0,0)
# robot_pose = (0,0,0)
pose = Quaternion()
## ![Initialize]

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timeStep) != -1:
    # Read the sensors:
    roll, pitch, heading = IMUsensor.getRollPitchYaw()
    xpos, altitude , zpos = GPSsensor.getValues()
    roll_vel, bleh, pitch_vel = GYROsensor.getValues()
    time += tsp

    ## [Publish true robot position]
    # true_pos.vector = Vector3(xpos, altitude , zpos)
    # true_pos.header.stamp.secs = time
    true_pos = Quaternion(xpos, altitude , zpos, time)
    true_pos_pub.publish(true_pos)
    ## ![Publish true robot position]

    ## [Publish sensors position]
    robot_pose = addNoise((xpos, altitude , zpos), robot_pose)
    # pose.vector = robot_pose
    # pose.header.stamp.secs = time
    pose = Quaternion(robot_pose[0], robot_pose[1], robot_pose[2], time)
    path_pub.publish(pose)
    ## ![Publish sensors position]

    ##[Derivate position for speed]
    littleTimeStep = timeStep/1000.0
    xSpeed = (xpos-xpos_old)/littleTimeStep
    ySpeed = (altitude-altitude_old)/littleTimeStep
    zSpeed = (zpos-zpos_old)/littleTimeStep
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

    radcoeff = 180.0/math.pi
    scaling = -1

    imu_pub.publish(Vector3((roll+initials[0])*radcoeff*scaling, (pitch+initials[1])*radcoeff*scaling, (heading+initials[2])*radcoeff*scaling))
    speed_pub.publish(Vector3(math.cos(heading)*xSpeed*-1+math.sin(heading)*zSpeed*-1,ySpeed,math.sin(heading)*xSpeed+math.cos(heading)*zSpeed))

    velocity_pub.publish(bleh)

    # if logger==True:
    #     f.write(str(xpos)+","+str(altitude)+","+str(zpos)+"\n")
    """
    Here it goes all the modes
    """
    ##[Control drive]
    drive, side = drive_side(heading_msg)
    ##![Control drive]

    fl_wheel=4*drive+4*side
    fr_wheel=4*drive-4*side
    rl_wheel=4*drive+4*side
    rr_wheel=4*drive-4*side

    front_left_motor.setVelocity(-clampval)#positive is up  #0.44467908653
    front_right_motor.setVelocity(clampval)#negative is up #0.44616503673
    rear_left_motor.setVelocity(clampval)#negative is up     #0.42589835641
    rear_right_motor.setVelocity(-clampval)#positive is up  #0.42744959936

    FL_wheel.setVelocity(fl_wheel)
    FR_wheel.setVelocity(fr_wheel)
    RL_wheel.setVelocity(rl_wheel)
    RR_wheel.setVelocity(rr_wheel)

    pass
