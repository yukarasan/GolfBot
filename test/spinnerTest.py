#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

from threading import Thread


# Ensure your necessary import statements are at the top of your file
# ...

class SpinnerThreadInwards(Thread):
    def __init__(self, spinner):
        Thread.__init__(self)
        self.spinner = spinner
        self.running = True
        self.rotation = 0

    def run(self):
        while self.running or self.rotation % 360 != 0:
            self.spinner.run_angle(speed=-300, rotation_angle=10)
            self.rotation += 10
            self.rotation %= 360

    def stop(self):
        self.running = False

    def stop_spinner(self):
        self.spinner.stop()


def main():
    # Initialize the EV3 Brick
    ev3 = EV3Brick()

    # Initialize the spinner motor
    spinner = Motor(Port.C)

    # Create and start the SpinnerThread
    spinner_thread = SpinnerThreadInwards(spinner)
    spinner_thread.start()

    # Let the spinner spin for 10 seconds
    wait(10000)

    # Stop the spinner thread
    spinner_thread.stop()

    # Wait until the spinner thread has fully stopped
    spinner_thread.join()


if __name__ == "__main__":
    main()
