"""UAV_controller controller."""

from controller import Robot
import numpy as np
import cv2
from gamepad_reader import Gamepad
from locomotion import Locomotion

# TODO
# 1. 换模型
# 2. 完成方向控制算法
# 3. 对接手柄控制
# 4. duplicate boats
# 设计对接算法：ArUco 识别、GPS 位姿估计
# 5. 实现 PID 对接
# 6. 分析：推进器力矩、对接碰撞力
# 7. 滤波算法对传感器降噪

def sensor_builder(name, timestep):
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    return sensor

use_gamepad = True
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


# GPS
gps = sensor_builder("gps", timestep)

# Gyro
gyro = sensor_builder("gyro", timestep)

# Inertial Unit
inertial_unit = sensor_builder("inertia_unit", timestep)

# Camera
camera = sensor_builder("camera", timestep)  # (240, 320, 3)
K = np.array([265, 0, 240, 0, 265, 160, 0, 0, 1]).reshape((3,3))
distCoeffs = np.zeros(5)
ARUCO_TAG = cv2.aruco.DICT_6X6_50
this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_TAG)
this_aruco_parameters = cv2.aruco.DetectorParameters_create()

# Gamepad
if use_gamepad:
    gamepad = Gamepad(1, 1, 1)

locomotion = Locomotion(robot, motors)

# Main loop:
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    if use_gamepad:
        lin_speed, ang_speed, e_stop = gamepad.get_command()
        locomotion.forward(lin_speed[0])
        locomotion.moveRight(lin_speed[1])
        locomotion.turnLeft(ang_speed)

    # for motor in motors:
    #     motor.setVelocity(-0.1)
        
    image = np.array(camera.getImageArray()).astype(np.uint8)  # (320, 480, 3)
    image = cv2.rotate(cv2.flip(image, 0), cv2.ROTATE_90_CLOCKWISE)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR).astype(np.uint8)
    
    if use_aruco:
        corners, ids, _ = cv2.aruco.detectMarkers(image, this_aruco_dictionary, parameters=this_aruco_parameters)
        #print(corners)

        if len(corners) > 0:
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.03, K, distCoeffs)
            #K, distCoeffs = cv2.aruco.calibrateCameraAruco(corners, ids, 3, board, (480, 320,))

            
    # cv2.imshow("window", image)
    # cv2.waitKey(1)
if use_gamepad:
    gamepad.stop()
# Enter here exit cleanup code.
