#!/usr/bin/env pybricks-micropython
from http.client import HTTPConnection
from urllib.parse import urlparse
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, DriveBase
from pybricks.tools import wait
from server.Logic.DetermineInstruction import Instructions

#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog

from threading import Thread
from server.Logic.DetermineInstruction import Instructions


def main():
    # Objects and setup
    ev3 = EV3Brick()
    left_wheel = Motor(Port.C)
    right_wheel = Motor(Port.B)

    # Wheel diameter and axle track (in millimeters)
    wheel_diameter = 56
    axle_track = 104

    # DriveBase object
    robot = DriveBase(left_wheel, right_wheel, wheel_diameter, axle_track)

    # IP to the server
    server_url = "http://10.209.234.177:8081/"

    while True:
        try:
            response = make_get_request(server_url)
            if response.status == 200:
                json_data = response.read()
                process_instruction(robot=robot, instruction=json_data)
            else:
                print('Request failed.')

        except Exception as e:
            print(e)


def make_get_request(url):
    parsed_url = urlparse(url)
    connection = HTTPConnection(parsed_url.netloc)
    connection.request("GET", parsed_url.path)
    response = connection.getresponse()
    return response


def process_instruction(robot: DriveBase, instruction):
    if instruction["instruction"] == Instructions.MOVE_LEFT.value:
        angle = float(instruction["angle"])
        turn(robot=robot, angle=angle)
    elif instruction["instruction"] == Instructions.MOVE_RIGHT.value:
        angle = float(instruction["angle"])
        turn(robot=robot, angle=angle)
    elif instruction["instruction"] == Instructions.MOVE_FORWARD.value:
        distance = float(instruction["distance"])
        move(robot=robot, distance=distance, robot_front_length=30)


def turn(robot: DriveBase, angle):
    # Code to turn the robot left
    print("Turning:", angle)
    # Adjust the motor rotation and speed according to your robot's setup
    robot.turn(angle=angle)


def move(robot: DriveBase, distance, robot_front_length):
    distance = distance - robot_front_length
    # Code to move the robot forward
    print("Moving forward:", distance)
    # Adjust the motor rotation and speed according to your robot's setup
    robot.straight(distance=distance)
    while robot.is_running():
        wait(10)


if __name__ == "__main__":
    main()
