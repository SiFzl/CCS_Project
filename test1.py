from controller import Robot, Motor, DistanceSensor, Camera, Receiver, Emitter, GPS, InertialUnit
import time
import struct
import numpy as np
import cv2
# import pandas as pd

robot = Robot()

timeStep = int(robot.getBasicTimeStep())  
delta_t = robot.getBasicTimeStep()/10
        
max_velocity = 6.28      # Set a maximum velocity time constant

#E_puck physical parameters
R = 0.0205               #Radius of the wheels
D = 0.0565               #Distance between the wheels
A = 0.05                 #Distance from the center of the wheels to the point of interest

#Colors
# holesColor = b'\n\n\n\xff' 
# swampColor = b'\x12\x1b \xff' 

holescolor = (0,0,0)

#Global Variables
last_e = 0.0
last_last_e = 0.0
last_u = 0.0

duration = 0 
startTime = 0
#Initial Values 
wl = 0.0
wr = 0.0

u = 0.0 
w = 0.0

x = 0.0
y = 0.0
phi = 0.0 

r = 0.0 #initial target angle

#Create objects for all robot sensors
#Internal Unit sensor
inertial_unit = robot.getDevice('inertial_unit')
inertial_unit.enable(timeStep)

#Distance Sensors
frontDist = robot.getDevice('ps0')
leftfrontdist = robot.getDevice('ps1')
rightfrontdist = robot.getDevice('ps2')
leftdist = robot.getDevice('ps3')
rightdist = robot.getDevice('ps4')

frontDist.enable(timeStep)
leftfrontdist.enable(timeStep)
rightfrontdist.enable(timeStep)
leftdist.enable(timeStep)
rightdist.enable(timeStep)

#Cameras
frontcamera = robot.getDevice('front_camera')
rightcamera = robot.getDevice('right_camera')
leftcamera = robot.getDevice('left_camera')

frontcamera.enable(timeStep)
rightcamera.enable(timeStep)
leftcamera.enable(timeStep)

camera = []
camera.append(frontcamera)
camera.append(rightcamera)
camera.append(leftcamera)

#Color Sensors
colorSensor = robot.getDevice("color_sensor")
colorSensor.enable(timeStep)

#Emitter and reciver
receiver = robot.getDevice("receiver") 
receiver.enable(timeStep)
emitter = robot.getDevice("emitter")    # Emitter doesn't need enable

#GPS Sensor
gps = robot.getDevice("gps")
gps.enable(timeStep)


#Motors
leftwheel = robot.getDevice('LW motor')       # Create an object to control the left wheel
rightwheel = robot.getDevice('RW motor')     # Create an object to control the right wheel

leftwheel.setPosition(float("inf"))
rightwheel.setPosition(float("inf"))

leftwheel.setVelocity(0.0)
rightwheel.setVelocity(0.0)

#Encoders
# encoderleft = robot.getDevice('LW sensor') 
# encoderright = robot.getDevice('RW sensor') 
# encoderleft.enable(timeStep)
# encoderright.enable(timeStep)


def linear_angular_2_wl_wr(u , w) :
    global D, R, wl, wr 
    wl = (2 * u + w * D) / (2 * R)
    wr = (2 * u - w * D) / (2 * R)

    return wl , wr

class PIDController :

    def __init__(self, T) :
        self.T = T 

    def set_kp(self , kp) : 
        return kp

    def set_kd(self , kd) :
        return kd

    def set_ki(self , ki) :
        return ki



    def compute_controller(self , kp , kd , ki , T , e) :
        global last_e
        global last_last_e 
        global last_u 
  
        u = (kp + 1/T * kd + 1/2 * ki * T) * e + (1/2 * ki * T - 2/T * kd - kp) * last_e + (1/T * kd) * last_last_e + last_u 

        return u

def goToAngle(e) :
    controller = PIDController(0.001)
    kp = controller.set_kp(1)
    kd = controller.set_kd(0.01)
    ki = controller.set_ki(0.8)
    u = controller.compute_controller(kp , kd , ki , 0.001 , e)
    print(f"goToAngle{u}")
    return u

def determine_next_move():  
    if isWallFront:
        if isWallLeft:
            return "right"
        elif isWallRight:
            return "left"
        elif isWallLeftFront:
            return "right"
        elif isWallRightFront:
            return "left"
        elif isWallRightFront and isWallLeftFront:
            return "back"
        
    elif isWallLeftFront:
        if not isWallRight:
            return "right"
        else:
            return "back"
    elif isWallRightFront:
        if not isWallLeft:
            return "left"
        else:
            return "back"
    # elif isWallFront and isWallLeft and isWallRight and isWallLeftFront and isWallRightFront:
    #     return "back"
    else:
        return "forward"

    

def update_target_angle(next_move):
    global duration, startTime, r, wl, wr
   
    # if RGB == (0,0,0):
    #     wl = -0.6 * max_velocity
    #     wr = -0.6 * max_velocity
    #     r = -np.pi
    
    if color < 80:
        # time.sleep(0.1)
        wl = -0.6 * max_velocity
        wr = -0.6 * max_velocity
        r = -np.pi
        next_move = "left"
    if next_move == "left":
        wl = -0.6 * max_velocity
        wr = 0.6 * max_velocity
        r = np.pi/2             # Turn left by setting target angle to 90 degrees
    elif next_move == "right":
        wl = 0.6 * max_velocity
        wr = -0.6 * max_velocity
        r = -np.pi/2            # Turn right by setting target angle to -90 degrees
    elif next_move == 'back':
        wl = -0.6 * max_velocity
        wr = -0.6 * max_velocity
        r = -np.pi
    elif next_move == 'leftFront':
        wl = -0.6 * max_velocity
        wr = 0.6 * max_velocity
        r = np.pi/4
    elif next_move == 'rightFront':
        wl = 0.6 * max_velocity
        wr = -0.6 * max_velocity
        r = -np.pi/4
    elif next_move == 'forward':
        wl = 0.6 * max_velocity
        wr = 0.6 * max_velocity
        r = 0
    return r, wl, wr

# def avoidHoles():
#     global duration, startTime
#     color = colorSensor.getImage()
#     if color == holesColor :
#         r = -np.pi
#         startTime = robot.getTime()
#         duration = 2
#     return r


########### MAIN LOOP #############

while True :
    while robot.step(timeStep) != -1:
        color = colorSensor.imageGetGray(colorSensor.getImage(), colorSensor.getWidth(), 0, 0)
        # red = colorSensor.imageGetRed(color, 1, 0, 0)
        # green = colorSensor.imageGetGreen(color, 1, 0, 0)
        # blue = colorSensor.imageGetBlue(color, 1, 0, 0)

        # RGB = (red, green, blue)

        # if robot.getTime() - startTime < duration :
        #     pass 
        # else :
        #     startTime = 0 
        #     duration = 0

        #Get the valuse of distance sensors and check the distance from the wall
        isWallFront = (frontDist.getValue()<0.1)
        isWallLeftFront = (leftfrontdist.getValue()<0.1)
        isWallRightFront = (rightfrontdist.getValue()<0.1)
        isWallLeft =  (leftdist.getValue() <0.1)
        isWallRight =  (rightdist.getValue() <0.1)

        #Controller 
        current_phi = inertial_unit.getRollPitchYaw()[2]
        print(inertial_unit.getRollPitchYaw()[2], inertial_unit.getRollPitchYaw()[1], inertial_unit.getRollPitchYaw()[0])
        e = r - current_phi
        u = goToAngle(e)
        # [wl , wr] = linear_angular_2_wl_wr(u,0)

        print(f'r={r}, current_phi={current_phi}')
        print(f'u:{u},e:{e},wl:{wl},wr:{wr},w:{w}')
        
        next_move = determine_next_move()
        print(f'{next_move}')
        [r, wl, wr] = update_target_angle(next_move)
        print(f'{r}')
        leftwheel.setVelocity(wl)
        rightwheel.setVelocity(wr)

        last_u = u
        last_last_e = last_e
        last_e = e

        time.sleep(0.001) 