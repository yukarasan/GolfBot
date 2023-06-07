#!/usr/bin/env pybricks-micropython
import requests
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

from threading import Thread
from server.Logic.DetermineInstruction import Instructions


# This program requires LEGO EV3 MicroPython v2.0 or higher.
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

    # Soundfile object to play the winning sound when the robot reaches the end
    winning_sound = SoundFile.FANFARE
    # ImageFile object to display the winning image when the robot reaches the end
    winning_image = ImageFile.THUMBS_UP

    # Ip to the server
    ip = "http://10.209.234.177:8081/"
    while True:
        try:
            response = requests.get(ip)
            if response.status_code == 200:
                json_data = response.json()
                process_instruction(robot=robot, instruction=json_data)
            else:
                print('Request failed.')

        except Exception as e:
            print(e)


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


def stop(robot: DriveBase):
    robot.stop()


def play_winning_sound(ev3: EV3Brick, soundfile):
    try:
        ev3.speaker.set_volume(100)
        ev3.speaker.play_file(soundfile)
    except Exception as e:
        print(f"Failed to play sound: {soundfile}. Error: {e}")


def display_winning_image(ev3: EV3Brick, imagefile):
    try:
        ev3.screen.load_image(imagefile)
    except Exception as e:
        print(f"Failed to display image: {imagefile}. Error: {e}")

if __name__ == "__main__":
    main()
