#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


def main():
    # Objects
    ev3 = EV3Brick()
    colorSensor = ColorSensor(Port.S1)

    # Test the color sensor
    while True:
        if red(colorSensor):
            print("Red")
            ev3.light.on(Color.RED)
        else:
            print("Green")
            ev3.light.on(Color.GREEN)


# Function that detects the red color
def red(colorSensor: ColorSensor):
    if colorSensor.color() == Color.RED:
        return True
    else:
        return False
