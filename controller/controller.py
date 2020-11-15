from controller import Robot
from controller import InertialUnit
from controller import Gyro
from controller import Motor
import math
import time
import rospy
import os

from geometry_msgs.msg import Vector3
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import String

##[Functions]

##[Callback]
def headingCallback(msg):
    global heading_msg
    heading_msg = msg
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
def drive_side(heading_msg):
    angle = heading_msg.y
    distance = heading_msg.x

    ##[Control drive]
    if abs(heading_angle) <= math.pi/2: # ahead of the robot
        if abs(heading_angle) > 5*math.pi/180: # 5º
            side = side_sign(heading_angle,True)

            drive = 0
            if goal_dist > 5 and abs(heading_angle) < math.pi/4: # 45º
                drive = 1
            elif goal_dist > 2 and abs(heading_angle) < 2*math.pi/18: # 20º
                drive = 1
        else:
            drive = 1
            if goal_dist < 1:
                side = side_sign(heading_angle,True)
            else:
                side = 0
    else:
        if abs(heading_angle) < (math.pi - 5*math.pi/180): # 5º
            side = side_sign(heading_angle,False)

            drive = 0
            if goal_dist > 5 and abs(heading_angle) > (math.pi - math.pi/4): # 45º
                drive = -1
            elif goal_dist > 2 and abs(heading_angle) > (math.pi - 2*math.pi/18): # 20º
                drive = -1
        else:
            drive = -1
            if goal_dist < 1:
                side = side_sign(heading_angle,False)
            else:
                side = 0
    ##![Control drive]

    return drive, side
## ![Drive - Side]

##![Functions]

##[Declare]
# create the Robot instance.
robot = Robot()

pitch_disturbance = 0
roll_disturbance = 0
yaw_disturbance = 0

# get the time step of the current world.
timeStep = int(robot.getBasicTimeStep())
# print("Time Step is: "+str(timeStep))

# Get and enable devices.
rospy.init_node('python_submarine_controller', anonymous=True) # node is called 'python_webots_controller'
# rospy.loginfo("Loading Webots Controller")

##[Publishers]
imu_pub = rospy.Publisher('imuValues', Vector3, queue_size=1000)
speed_pub = rospy.Publisher("headingSpeed",Float32,queue_size=10)
velocity_pub = rospy.Publisher('submarineVelocity', Vector3, queue_size=10)
global_pos_pub = rospy.Publisher("robot_pose", Vector3, queue_size=1000)
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
global_pos = Vector3(xpos, altitude, zpos)
global_pos_pub.publish(global_pos)
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
##![Declare]

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timeStep) != -1:
    # Read the sensors:
    roll, pitch, heading = IMUsensor.getRollPitchYaw()
    xpos, altitude , zpos = GPSsensor.getValues()
    roll_vel, bleh, pitch_vel = GYROsensor.getValues()

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

    radcoeff = 180.0/math.pi
    scaling = -1

    imu_pub.publish(Vector3(roll*radcoeff*scaling,pitch*radcoeff*scaling,heading*radcoeff*scaling))
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

    front_left_motor.setVelocity(CLAMP(front_left_motor_input,-clampval,clampval))#positive is up  #0.44467908653
    front_right_motor.setVelocity(CLAMP(-front_right_motor_input,-clampval,clampval))#negative is up #0.44616503673
    rear_left_motor.setVelocity(CLAMP(-rear_left_motor_input,-clampval,clampval))#negative is up     #0.42589835641
    rear_right_motor.setVelocity(CLAMP(rear_right_motor_input,-clampval,clampval))#positive is up  #0.42744959936

    FL_wheel.setVelocity(fl_wheel)
    FR_wheel.setVelocity(fr_wheel)
    RL_wheel.setVelocity(rl_wheel)
    RR_wheel.setVelocity(rr_wheel)

    pass
