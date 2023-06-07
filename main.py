#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

from threading import Thread
import socket
import json

# This program requires LEGO EV3 MicroPython v2.0 or higher.
def main():
    # Objects and setup
    ev3 = EV3Brick()
    left_wheel = Motor(Port.C)
    right_wheel = Motor(Port.B)
    conveyor = Motor(Port.D)

    # Wheel diameter and axle track (in millimeters)
    wheel_diameter = 56
    axle_track = 100

    # DriveBase object
    robot = DriveBase(left_wheel, right_wheel, wheel_diameter, axle_track)

    # Soundfile object to play the winning sound when the robot reaches the end
    winning_sound = SoundFile.FANFARE
    # ImageFile object to display the winning image when the robot reaches the end
    winning_image = ImageFile.THUMBS_UP

    while True:
        # Running the conveyor belt
        # conveyor.run_time(1000, 1000)

        if is_moving(left_wheel, right_wheel):
            continue
        else:

            # Socket connection setup
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # 10.209.234.177
            sock.connect(("172.20.10.4", 8081))

            try:
                # Send request
                request = "GET / HTTP/1.1\r\nHost: 10.209.234.177\r\n\r\n"
                sock.send(request.encode())

                # Read response
                response = ''
                while True:
                    data = sock.recv(1024)
                    if not data:
                        break
                    response += data.decode()

                # Parse JSON if response is 200
                header, _, body = response.partition('\r\n\r\n')
                if '200 OK' in header:
                    json_data = json.loads(body)
                    print(json_data)
                    process_instruction(robot=robot, instruction=json_data)
                else:
                    print('Request failed.')

            except Exception as e:
                print(e)

            finally:
                sock.close()


# Custom function to check if the robot is moving
def is_moving(left_wheel: Motor, right_wheel: Motor):
    return abs(left_wheel.speed()) > 0 or abs(right_wheel.speed()) > 0

def process_instruction(robot: DriveBase, instruction):
    if instruction["instruction"] == "Left":
        angle = float(instruction["angle"])
        turn(robot=robot, angle=angle)
    elif instruction["instruction"] == "Right":
        angle = float(instruction["angle"])
        turn(robot=robot, angle=angle)
    elif instruction["instruction"] == "Forward":
        distance = float(instruction["distance"])
        move(robot=robot, distance=distance)


def turn(robot: DriveBase, angle):
    # Code to turn the robot left
    print("Turning:", angle)
    # Adjust the motor rotation and speed according to your robot's setup
    robot.turn(angle=angle)


def move(robot: DriveBase, distance):
    # Code to move the robot forward
    print("Moving forward:", distance)
    # Adjust the motor rotation and speed according to your robot's setup
    robot.straight(distance=distance * 10) # Convert to millimeters


def stop(robot: DriveBase):
    robot.stop()


def play_winning_sound(ev3: EV3Brick, soundfile):
    try:
        ev3.speaker.set_volume(100)
        ev3.speaker.play_file(soundfile)
    except Exception as e:
        print("Failed to play sound.")


def display_winning_image(ev3: EV3Brick, imagefile):
    try:
        ev3.screen.load_image(imagefile)
    except Exception as e:
        print("Failed to display image.")

if __name__ == "__main__":
    main()
