#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile 

import socket
from threading import Thread

# This program requires LEGO EV3 MicroPython v2.0 or higher.

class SharedState:
    def __init__(self):
        self.conveyor_belt_running = True

def main(): 
    # Objects and setup
    ev3 = EV3Brick()
    left_wheel = Motor(Port.C)
    right_wheel = Motor(Port.B)
    conveyor = Motor(Port.D)
    
    # Wheel diameter and axle track (in millimeters)
    wheel_diameter = 56
    axle_track = 104
    
    # DriveBase object
    robot = DriveBase(left_wheel, right_wheel, wheel_diameter, axle_track)
    
    # Global variables
    conveyor_belt_running = True

    # Custom function to check if the robot is moving
    def is_moving():
        return abs(left_wheel.speed()) > 0 or abs(right_wheel.speed()) > 0
    
    # Pick balls function
    def conveyor_belt(state):
        print("Conveyor")
        while state.conveyor_belt_running:
            conveyor.run_angle(speed=500, rotation_angle=-10000, wait=False)
            wait(10)  # Add a small delay to prevent high CPU usage

    # Drive straight
    def drive_straight(state):
        print("Straight")
        robot.straight(5000)
        while is_moving():
            wait(10)
        state.conveyor_belt_running = False  # Stop the conveyor_belt loop when the robot stops moving

    try: 
        # Creating a socket object
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
        # connect to the server at the specified IP address and port
        client_socket.connect(("172.20.10.4", 8081))

        # create a file-like wrapper for receiving data from the server
        in_file = client_socket.makefile('r')

        # read the server's response
        response = in_file.readline()
        print("Received:", response.strip())

        # Execute instructions received from the server
        if response.strip() == 'pick_balls':
            shared_state = SharedState()
            thread1 = Thread(target=conveyor_belt, args=(shared_state,))
            thread2 = Thread(target=drive_straight, args=(shared_state,))
            thread1.start()
            thread2.start()
            print("Drive straight")
        elif response.strip() == 'drive_straight':
            robot.straight(500)
            while is_moving():
                wait(10)

        # close the input stream and socket
        in_file.close()
        client_socket.close()
    
    except Exception as e:
        print(e)

if __name__ == "__main__":
    main()