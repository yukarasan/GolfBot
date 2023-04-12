#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# This program requires LEGO EV3 MicroPython v2.0 or higher.

# Objects.
ev3 = EV3Brick()

left_wheel = Motor(Port.C)
right_wheel = Motor(Port.B)

conveyor = Motor(Port.D)

# Wheel diameter and axle track (in millimeters)
wheel_diameter = 56
axle_track = 104

# Create a DriveBase object
robot = DriveBase(left_wheel, right_wheel, wheel_diameter, axle_track)

# Custom function to check if the robot is moving
def is_moving():
    return abs(left_wheel.speed()) > 0 or abs(right_wheel.speed()) > 0

# Pick balls function
def pick_balls():
    conveyor.run_angle(speed=500, rotation_angle=-10000, wait=False)

# Set the robot's turn speed (in degrees/s)
turn_speed = 200

# Drive robot
pick_balls()
robot.straight(2000)
while is_moving():
    wait(10)