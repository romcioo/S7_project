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
imu_pub = rospy.Publisher('imuValues', Vector3, queue_size=1000)
speed_pub = rospy.Publisher("headingSpeed",Float32,queue_size=10)
velocity_pub = rospy.Publisher('submarineVelocity', Vector3, queue_size=10)
global_pos_pub = rospy.Publisher("robot_pose", Vector3, queue_size=1000)

IMUsensor = robot.getInertialUnit('inertial unit')  # front central proximity sensor
IMUsensor.enable(timeStep)

GPSsensor = robot.getGPS('gps')
GPSsensor.enable(timeStep)

GYROsensor = robot.getGyro("gyro")
GYROsensor.enable(timeStep)

# set robot initial values
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

def CLAMP(value, low, high):
    if value < low:
        return low
    elif value > high:
        return high
    return value

robot.step(timeStep)
xpos, altitude , zpos = GPSsensor.getValues()
global_pos = Vector3(xpos, altitude, zpos)
global_pos_pub.publish(global_pos)
xpos_old=xpos
altitude_old=altitude
zpos_old=zpos

drone_mode = True
atache_mode = False
car_mode = False
follow_mode = False
altitude_bool = False
angle_dist = 0.785398
angle_lock = -2.0944
logger=False
pi = math.pi

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timeStep) != -1:
    # Read the sensors:
    roll, pitch, heading = IMUsensor.getRollPitchYaw()
    xpos, altitude , zpos = GPSsensor.getValues()
    roll_vel, bleh, pitch_vel = GYROsensor.getValues()
    #print(str(roll_vel)+"\t"+str(pitch_vel))
    littleTimeStep = timeStep/1000.0
    xSpeed=(xpos-xpos_old)/littleTimeStep
    ySpeed=(altitude-altitude_old)/littleTimeStep
    zSpeed=(zpos-zpos_old)/littleTimeStep
    #print(str(xSpeed)+"\t"+str(ySpeed)+"\t"+str(zSpeed))
    xpos_old=xpos
    altitude_old=altitude
    zpos_old=zpos
    #  val = ds.getValue()
    fl_wheel=0
    fr_wheel=0
    rl_wheel=0
    rr_wheel=0

    radcoeff = 180.0/pi
    scaling = -1

    imu_pub.publish(Vector3(roll*radcoeff*scaling,pitch*radcoeff*scaling,heading*radcoeff*scaling))
    speed_pub.publish(Vector3(math.cos(heading)*xSpeed*-1+math.sin(heading)*zSpeed*-1,ySpeed,math.sin(heading)*xSpeed+math.cos(heading)*zSpeed))

    velocity_pub.publish(bleh)

    if logger==True:
        f.write(str(xpos)+","+str(altitude)+","+str(zpos)+"\n")
    if (drone_mode==True):
        roll_input = k_roll_p * (angle_dist*side-roll) - k_roll_d*roll_vel
        pitch_input = (k_pitch_p *(angle_dist*drive-pitch) - k_pitch_d*pitch_vel)
        if abs(roll_input)>20 or abs(pitch_input)>20:
            yaw_stop=0
        if (yaw_disturbance>(math.pi)):
            yaw_disturbance=yaw_disturbance-2*math.pi
        elif (yaw_disturbance<-(math.pi)):
            yaw_disturbance=yaw_disturbance+2*math.pi
        yaw_error=yaw_disturbance-heading
        if (yaw_error>(math.pi)):
            yaw_error=yaw_error-2*math.pi
        elif (yaw_error<(-math.pi)):
            yaw_error=yaw_error+2*math.pi
        yaw_input = yaw_stop*(k_yaw_p*yaw_error- k_yaw_d*bleh);

        vertical_input = k_vertical_p *CLAMP(target_altitude - altitude, -2.0, 2.0)-k_vertical_d*ySpeed;
        if roll>math.pi/2 or roll<-math.pi/2:
            vertical_input=-vertical_input
        if altitude_bool==True:
            target_altitude=altitude
            altitude_bool=False

        front_left_motor_input =k_vertical_thrust + vertical_input + roll_input + (0.321/0.246)*pitch_input - yaw_input
        front_right_motor_input=k_vertical_thrust + vertical_input - roll_input + (0.321/0.246)*pitch_input + yaw_input
        rear_left_motor_input  =k_vertical_thrust + vertical_input + roll_input - (0.321/0.246)*pitch_input + yaw_input
        rear_right_motor_input =k_vertical_thrust + vertical_input - roll_input - (0.321/0.246)*pitch_input - yaw_input

    elif atache_mode==True:

        if not(abs(drive) or abs(side)):
            vertical_input = k_vertical_p *CLAMP(target_altitude - altitude, -2.0, 2.0)-k_vertical_d*ySpeed;
            if roll>math.pi/2 or roll<-math.pi/2:
                vertical_input=-vertical_input
            if altitude_bool==True:
                target_altitude=altitude
                altitude_bool=False
            lock_on=1
        else:
            vertical_input=0
            lock_on=-2
        roll_input = k_roll_p * (angle_lock*side-roll) - k_roll_d*roll_vel
        pitch_input = (k_pitch_p *(-1.48353*drive-pitch) - k_pitch_d*pitch_vel)

        front_left_motor_input =lock_on*k_vertical_thrust + vertical_input + roll_input + (0.321/0.246)*pitch_input
        front_right_motor_input=lock_on*k_vertical_thrust + vertical_input - roll_input + (0.321/0.246)*pitch_input
        rear_left_motor_input  =lock_on*k_vertical_thrust + vertical_input + roll_input - (0.321/0.246)*pitch_input
        rear_right_motor_input =lock_on*k_vertical_thrust + vertical_input - roll_input - (0.321/0.246)*pitch_input

    elif car_mode==True:
        front_left_motor_input =-2*k_vertical_thrust-car_trun*400
        front_right_motor_input=-2*k_vertical_thrust+car_trun*400
        rear_left_motor_input  =-2*k_vertical_thrust+car_trun*400
        rear_right_motor_input =-2*k_vertical_thrust-car_trun*400
        fl_wheel=4*drive+4*side
        fr_wheel=4*drive-4*side
        rl_wheel=4*drive+4*side
        rr_wheel=4*drive-4*side
        #print(str(k_vertical_thrust)+"  \t"+str(roll_input)

    # OlaMark
    elif follow_mode == True:
        front_left_motor_input =-2*k_vertical_thrust-car_trun*400
        front_right_motor_input=-2*k_vertical_thrust+car_trun*400
        rear_left_motor_input  =-2*k_vertical_thrust+car_trun*400
        rear_right_motor_input =-2*k_vertical_thrust-car_trun*400
        #turning
        drive = 1
        side = 0
        # for x in range(10):
        #     fl_wheel=4*drive+4*side
        #     fr_wheel=4*drive-4*side
        #     rl_wheel=4*drive+4*side
        #     rr_wheel=4*drive-4*side
        #Straight line
        drive = -3
        side = 0
        for x in range(10):
            fl_wheel=4*drive+4*side
            fr_wheel=4*drive-4*side
            rl_wheel=4*drive+4*side
            rr_wheel=4*drive-4*side
    else :
        front_left_motor_input =2*k_vertical_thrust
        front_right_motor_input=2*k_vertical_thrust
        rear_left_motor_input  =2*k_vertical_thrust
        rear_right_motor_input =2*k_vertical_thrust

    clampval = 10000
    #print(str(front_left_motor_input)+"\t"+str(front_right_motor_input)+"\t"+str(rear_left_motor_input)+"\t"+str(rear_right_motor_input))
    if front_left_motor_input>1000 or front_right_motor_input>1000 or rear_left_motor_input>1000 or rear_right_motor_input>1000:
        print("motor input maxed: "+str(int(time.time())))
    front_left_motor.setVelocity(CLAMP(front_left_motor_input,-clampval,clampval))#positive is up  #0.44467908653
    front_right_motor.setVelocity(CLAMP(-front_right_motor_input,-clampval,clampval))#negative is up #0.44616503673
    rear_left_motor.setVelocity(CLAMP(-rear_left_motor_input,-clampval,clampval))#negative is up     #0.42589835641
    rear_right_motor.setVelocity(CLAMP(rear_right_motor_input,-clampval,clampval))#positive is up  #0.42744959936
    #fly_wheel.setVelocity(spin_boy)
    FL_wheel.setVelocity(fl_wheel)
    FR_wheel.setVelocity(fr_wheel)
    RL_wheel.setVelocity(rl_wheel)
    RR_wheel.setVelocity(rr_wheel)
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)


    pass
