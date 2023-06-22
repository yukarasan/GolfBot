#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


def main():
    ev3 = EV3Brick()
    colorSensor = ColorSensor(Port.S1)

    # Test the color sensor
    while True:
        wait(500)   
        if red(colorSensor):
            print("Red")
            ev3.light.on(Color.RED)
        else:
            print("Green")
            ev3.light.on(Color.GREEN)

def red(colorSensor: ColorSensor):
    if colorSensor.color() == Color.RED:
        return True
    else:
        return False

if __name__ == "__main__":
    main()