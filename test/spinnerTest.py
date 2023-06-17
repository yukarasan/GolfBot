#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

from threading import Thread

class SpinnerThreadInwards(Thread):
    def __init__(self, spinner):
        Thread.__init__(self)
        self.spinner = spinner
        self.running = True
        self.rotation = 360  # one full rotation

    def run(self):
        while self.running:
            self.spinner.run_angle(speed=-300, rotation_angle=self.rotation, then=Stop.COAST, wait=True)

    def stop(self):
        self.running = False
        self.spinner.stop()  # Stops the spinner motor

def main():
    # Initialize the EV3 Brick
    ev3 = EV3Brick()

    # Initialize the spinner motor
    spinner = Motor(Port.C)

    # Create and start the SpinnerThread
    spinner_thread = SpinnerThreadInwards(spinner)
    spinner_thread.start()

    numOfLoop = 0
    while spinner_thread.running and numOfLoop < 5:
        wait(1000)  
        numOfLoop += 1
        print(numOfLoop)

    if spinner_thread.running:
        spinner_thread.stop()
    
    wait(10000)  # Wait for 10 seconds

if __name__ == "__main__":
    main()