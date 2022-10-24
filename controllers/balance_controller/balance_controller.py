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
gps = sensor_builder("gps_station")
imu = sensor_builder("inertial_unit_station")

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
pd_x = PID_Controller(10, 10, 0.0)
pd_y = PID_Controller(10, 10, 0.0)
pd_yaw = PID_Controller(5, 0.5, 0.0)
DX_TARGET = 0.4
DX_MOVE_BACK = 1
DY_THRESHOLD = 0.1
DY_OFFSET = 0.016
PHI_THRESHOLD = 2/180*math.pi
robot.step(timestep)
POS = gps.getValues()
YAW = imu.getRollPitchYaw()[-1]

# Main loop:
while robot.step(timestep) != -1:

    if use_gamepad:
        lin_speed, ang_speed, e_stop = gamepad.get_command()
    else:
        lin_speed, ang_speed, e_stop = teleop.get_command()

    positions = gps.getValues()
    rpy = imu.getRollPitchYaw()

    lin_speed[0] = -pd_x.apply(positions[0]-POS[0])
    lin_speed[1] = -pd_y.apply(positions[2]-POS[2])
    ang_speed = -pd_yaw.apply(rpy[-1] - YAW)

    locomotion.forward(lin_speed[0])
    locomotion.moveLeft(lin_speed[1])
    locomotion.turnRight(ang_speed)

if use_gamepad:
    gamepad.stop()
else:
    keyboard.disable()
# Enter here exit cleanup code.
