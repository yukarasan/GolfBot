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
    motorA.run(500)
    motorB.run(500)

    while not touchSensor.pressed():
        wait(10)  

    motorA.stop()
    motorB.stop()

    print("Touch sensor pressed, motors stopped.")

def is_touch_sensor_pressed(touch_sensor: TouchSensor):
    return touch_sensor.pressed()

if __name__ == "__main__":
    main()
