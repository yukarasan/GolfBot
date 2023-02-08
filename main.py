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

left_wheel = Motor(Port.B)
right_wheel = Motor(Port.C)

fork = Motor(Port.D)

robot = DriveBase(left_wheel, right_wheel, wheel_diameter = 70, axle_track = 100)


# Program.
# notes = ['C4/4', 'C4/4', 'G4/4', 'G4/4', 'A4/4', 'A4/4', 'G4/4', 'F4/4', 'F4/4', 'E4/4', 'E4/4', 'D4/4', 'D4/4', 'C4/4']
# ev3.speaker.play_notes(notes, tempo = 120)


robot.straight(1000)

# Wave
fork.run_angle(speed = 2000, rotation_angle = 150)
fork.run_angle(speed = 2000, rotation_angle = -150)
fork.run_angle(speed = 2000, rotation_angle = 150)
fork.run_angle(speed = 2000, rotation_angle = -150)