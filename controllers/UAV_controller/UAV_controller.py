"""UAV_controller controller."""
import os
import inspect

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
os.sys.path.insert(0, parentdir)

from controller import Robot
import cv2
import math
import numpy as np
from gamepad_reader import Gamepad
from locomotion import Locomotion
from teleop_control import Teleop
from pid_control import PID_Controller
from moving_window_filter import MovingWindowFilter
from orientation_tools import rot_to_rpy

# TODO
# TODO 写一下 3PD 控制和GPS+IMU 融合卡尔曼滤波
# TODO 直接滤波传感器吧，然后做几组不同初始位置到达预定对接地点的视频和数据图


def sensor_factory(robot, timestep):
    def sensor_builder(name):
        sensor = robot.getDevice(name)
        sensor.enable(timestep)
        return sensor

    return sensor_builder

def near_zero(val):
    if abs(val) < 1e-3:
        return 0
    else:
        return val

use_gamepad = False
use_aruco = True
DTYPE = np.float32

# create the Robot instance.
robot = Robot()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Motors
motor_name = ["font_motor", "rear_motor", "left_motor", "right_motor"]
motors = []
for name in motor_name:
    motor = robot.getDevice(name)
    motor.setPosition(float('+inf'))  # Velocity control mode.
    motor.setVelocity(0.0)
    motors.append(robot.getDevice(name))

# Sensors
sensor_builder = sensor_factory(robot, timestep)
gps = sensor_builder("gps")
gyro = sensor_builder("gyro")
inertial_unit = sensor_builder("inertia_unit")
camera = sensor_builder("camera")  # (240, 320, 3)

# Camera params
cx = camera.getWidth() / 2
cy = camera.getHeight() / 2
# f = camera.getFocalLength()
f = cx/math.tan(camera.getFov()/2)
cameraMatrix = np.array([f, 0, cx, 0, f, cy, 0, 0, 1], dtype=DTYPE).reshape((3,3))
distCoeffs = np.zeros(4, dtype=DTYPE)
ARUCO_TAG = cv2.aruco.DICT_6X6_50
aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_TAG)
aruco_parameters = cv2.aruco.DetectorParameters_create()

# Gamepad and Keyboard
if use_gamepad:
    gamepad = Gamepad(1, 1, 1)
else:
    keyboard = robot.getKeyboard()
    keyboard.enable(timestep)
    teleop = Teleop(robot, keyboard)

# Locomotion
locomotion = Locomotion(robot, motors)
lin_speed = [0, 0, 0]
ang_speed = 0
e_stop = False

# PID
pd_dx = PID_Controller(1, 0.1, 0.0)
pd_dy = PID_Controller(2, 0.5, 0.0)
pd_phi = PID_Controller(0.8, 0.5, 0.0)
DX_TARGET = 0.4
DX_MOVE_BACK = 1
DY_THRESHOLD = 0.1
DY_OFFSET = 0.016
PHI_THRESHOLD = 2/180*math.pi


# Main loop:
while robot.step(timestep) != -1:

    if use_gamepad:
        lin_speed, ang_speed, e_stop = gamepad.get_command()
    else:
        lin_speed, ang_speed, e_stop = teleop.get_command()

    # (320, 480, 3)
    image = np.frombuffer(camera.getImage(), np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))[:,:,:3].copy()
    
    if use_aruco:
        corners, ids, _ = cv2.aruco.detectMarkers(image, aruco_dictionary, parameters=aruco_parameters)
        # print(corners)

        if len(corners) > 0:
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.03, cameraMatrix, distCoeffs)
            rot_mat, _ = cv2.Rodrigues(rvecs[0]) # (3,3)
            rpy = rot_to_rpy(rot_mat)
            yaw = float(rpy[-1])
            distance1 = tvecs[0].squeeze() # (3,)
            # distance = (rot_mat @ distance).squeeze()
            # print(distance.shape)
            dx = near_zero(distance1[-1] - DX_TARGET)
            dy = near_zero(distance1[0]) 
            phi = near_zero(math.atan2(dy, dx))

            lin_speed[1] = -pd_dy.apply(dy)
            lin_speed[0] = -pd_dx.apply(dx)
            # print("dx:", dx)
            # print("dy:", dy)
            # print("phi:", phi)
            if dx>0:
                if dy<DY_THRESHOLD or phi < PHI_THRESHOLD:
                    ang_speed = -pd_phi.apply(yaw)
                    # ang_speed = -pd_phi.apply(phi)
                    lin_speed[0] = -pd_dx.apply(dx)
                else:
                    lin_speed[0] = -pd_dx.apply(dx-DX_MOVE_BACK)

    locomotion.forward(lin_speed[0])
    locomotion.moveLeft(lin_speed[1])
    locomotion.turnRight(ang_speed)

    # cv2.imshow("window", image)
    # cv2.waitKey(1)
if use_gamepad:
    gamepad.stop()
else:
    keyboard.disable()
# Enter here exit cleanup code.
