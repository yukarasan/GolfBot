#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor)
from pybricks.parameters import Port
from pybricks.tools import wait

# Objects and setup
ev3 = EV3Brick()
motorA = Motor(Port.A)
motorB = Motor(Port.B)
touchSensor = TouchSensor(Port.S1)

def main():
    # Start the motors
    motorA.run(500)   # Speed is in degrees per second
    motorB.run(500)

    # Loop until the touch sensor is pressed
    while not touchSensor.pressed():
        wait(10)  # Wait 10ms to prevent the CPU from getting overloaded

    # Stop the motors when the touch sensor is pressed
    motorA.stop()
    motorB.stop()

    # Provide feedback
    print("Touch sensor pressed, motors stopped.")


def is_touch_sensor_pressed(touch_sensor: TouchSensor):
    """
    Returns True if the touch sensor is pressed, False otherwise.
    """
    return touch_sensor.pressed()

if __name__ == "__main__":
    main()
