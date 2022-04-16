"""UAV_controller controller."""

from controller import Robot
import numpy as np
import cv2
from gamepad_reader import Gamepad
from locomotion import Locomotion
from teleop_control import Teleop

# TODO
# 1. 换模型
# 2. 完成方向控制算法
# 3. 对接手柄控制
# 4. duplicate boats
# 设计对接算法：
# ArUco 识别
# GPS 位姿估计
# 读论文, 了解下算法
# 5. 实现 PID 对接
# 6. 分析：推进器力矩、对接碰撞力
# 7. 滤波算法对传感器降噪


def sensor_factory(robot, timestep):
    def sensor_builder(name):
        sensor = robot.getDevice(name)
        sensor.enable(timestep)
        return sensor

    return sensor_builder


use_gamepad = False
use_aruco = False

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
f = camera.getFocalLength()
cx = camera.getWidth() / 2
cy = camera.getHeight() / 2
K = np.array([f, 0, cx, 0, f, cy, 0, 0, 1]).reshape((3,3))
distCoeffs = np.zeros(5)
ARUCO_TAG = cv2.aruco.DICT_6X6_50
this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_TAG)
this_aruco_parameters = cv2.aruco.DetectorParameters_create()

# Gamepad and Keyboard
if use_gamepad:
    gamepad = Gamepad(1, 1, 1)
else:
    keyboard = robot.getKeyboard()
    keyboard.enable(timestep)
    teleop = Teleop(robot, keyboard)

# Locomotion
locomotion = Locomotion(robot, motors)
lin_speed = [0,0,0]
ang_speed = 0
e_stop = False

# Main loop:
while robot.step(timestep) != -1:

    if use_gamepad:
        lin_speed, ang_speed, e_stop = gamepad.get_command()
    else:
        lin_speed, ang_speed, e_stop = teleop.get_command()

    locomotion.forward(lin_speed[0])
    locomotion.moveLeft(lin_speed[1])
    locomotion.turnRight(ang_speed)

    # (320, 480, 3)
    image = np.frombuffer(camera.getImage(), np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))[:,:,:3].copy()
    
    if use_aruco:
        corners, ids, _ = cv2.aruco.detectMarkers(image, this_aruco_dictionary, parameters=this_aruco_parameters)
        # print(corners)

        if len(corners) > 0:
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.03, K, distCoeffs)
            # K, distCoeffs = cv2.aruco.calibrateCameraAruco(corners, ids, 3, board, (480, 320,))

            
    # cv2.imshow("window", image)
    # cv2.waitKey(1)
if use_gamepad:
    gamepad.stop()
else:
    keyboard.disable()
# Enter here exit cleanup code.
