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
import time

# Global direction counter to keep track of how many times the robot has turned left or right
direction_counter = 0
stopInstructions = False
going_to_goal = 0


class FourWheelMechanism(Thread):
    def __init__(self, four_wheel_mechanism):
        Thread.__init__(self)
        self.four_wheel_mechanism = four_wheel_mechanism
        self.running = True

    def run(self):
        while self.running:
            self.four_wheel_mechanism.run(1000)  # Run the conveyor belt
            time.sleep(0.1)  # Sleep for a short while to not hog the CPU

    def stop(self):
        self.running = False

    def stop_four_wheel_mechanism(self):
        self.four_wheel_mechanism.stop()  # Stops the conveyor motor


class SpinnerThreadInwards(Thread):
    def __init__(self, spinner):
        Thread.__init__(self)
        self.spinner = spinner
        self.running = True

    def run(self):
        while self.running:
            self.spinner.run(speed=-300)  # Make the spinner run continuously

    def stop(self):
        self.running = False
        self.spinner.stop()  # Stops the spinner motor


class SpinnerThreadOutward(Thread):
    def __init__(self, spinner):
        Thread.__init__(self)
        self.spinner = spinner
        self.running = True

    def run(self):
        while self.running:
            self.spinner.run(speed=300)  # Make the spinner run continuously

    def stop(self):
        self.running = False
        self.spinner.stop()  # Stops the spinner motor


# This program requires LEGO EV3 MicroPython v2.0 or higher.
def main():
    # Objects and setup
    ev3 = EV3Brick()
    left_wheel = Motor(Port.A)
    right_wheel = Motor(Port.B)
    four_wheel_mechanism = Motor(Port.D)
    spinner = Motor(Port.C)
    ultrasonic = UltrasonicSensor(Port.S1)

    # Wheel diameter and axle track (in millimeters)
    wheel_diameter = 56
    axle_track = 30

    # DriveBase object
    robot = DriveBase(left_wheel, right_wheel, wheel_diameter, axle_track)

    # Soundfile object to play the winning sound when the robot reaches the end
    winning_sound = SoundFile.FANFARE
    # ImageFile object to display the winning image when the robot reaches the end
    winning_image = ImageFile.THUMBS_UP

    # Start the four_wheel_mechanism belt thread
    four_wheel_mechanism_thread = FourWheelMechanism(four_wheel_mechanism)
    four_wheel_mechanism_thread.start()

    # Start the spinner thread
    spinner_thread = SpinnerThreadInwards(spinner)
    spinner_thread.start()

    while stopInstructions is not True:
        # Socket connection setup
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 10.209.234.177 || 172.20.10.4
        sock.connect(("192.168.1.215", 8081))

        # Get the distance to the nearest object
        distance = ultrasonic.distance()
        print("Distance to wall:", distance)

        # Distance to stop at (4 cm)
        stop_distance = 40
        reverse_distance = -100  # Distance to move backwards (10 cm)

        # If an object is detected within the stop distance, stop and move backwards
        if distance <= stop_distance or distance > 2000:
            wait(500)  # Wait for 1 second
            robot.straight(reverse_distance)
        else:
            try:
                # Send request
                request = "GET / HTTP/1.1\r\nHost: 192.168.1.215\r\n\r\n"
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
                    process_instruction(
                        robot=robot,
                        instruction=json_data,
                        spinner=spinner,
                        four_wheel_mechanism=four_wheel_mechanism,
                        four_wheel_mechanism_thread=four_wheel_mechanism_thread,
                        spinner_thread=spinner_thread,
                        distance_to_wall=distance
                    )
                else:
                    print('Request failed.')

            except Exception as e:
                print(e)

            finally:
                sock.close()

    if spinner_thread.running:
        spinner_thread.stop()

    # Stop the four_wheel_mechanism belt thread
    four_wheel_mechanism.stop()

    # TODO: Remember to also stop the inwards spinner thread

    wait(1000)  # Wait for 1 second
    ev3.speaker.play_file(winning_sound)  # Play the winning sound
    ev3.screen.load_image(winning_image)  # Display the winning image
    wait(10000)  # Wait for 10 seconds


def process_instruction(
        robot: DriveBase,
        instruction,
        spinner: Motor,
        four_wheel_mechanism: Motor,
        four_wheel_mechanism_thread: FourWheelMechanism,
        spinner_thread: SpinnerThreadInwards,
        distance_to_wall: int
):
    # Distance to stop at (4 cm)
    stop_distance = 40
    reverse_distance = -100  # Distance to move backwards (10 cm)

    # if instruction "go to goal" is "yes" then stop the spinner
    if instruction["go to goal"] == "yes":
        global going_to_goal
        going_to_goal += 1
        if going_to_goal == 3:
            spinner_thread.stop()
            going_to_goal = 0
    else:
        going_to_goal = 0

    if instruction["instruction"] in ["Left", "Right"]:

        global direction_counter
        direction_counter += 1

        print("Direction counter:", direction_counter)

        if direction_counter >= 20:
            direction_counter = 0

            if distance_to_wall <= stop_distance or distance_to_wall > 2000:
                wait(500)  # Wait for 1 second
                robot.straight(reverse_distance)
            else:
                move(robot=robot, distance=-10)  # move backwards
    else:
        direction_counter = 0  # Reset the counter if the instruction is not "Left" or "Right"

    if instruction["instruction"] == "Left":

        angle = float(instruction["angle"])
        distance = float(instruction["distance"])

        if distance_to_wall <= stop_distance or distance_to_wall > 2000:
            wait(500)  # Wait for 1 second
            robot.straight(reverse_distance)
        else:
            # If the distance is under 15, move backwards instead
            if distance < 0 and abs(angle) > 80:
                move(robot=robot, distance=reverse_distance)

            # Turn the robot left
            turn(robot=robot, angle=angle)

    elif instruction["instruction"] == "Right":
        angle = float(instruction["angle"])
        distance = float(instruction["distance"])

        if distance_to_wall <= stop_distance or distance_to_wall > 2000:
            wait(500)  # Wait for 1 second
            robot.straight(reverse_distance)
        else:

            # If the distance is under 15, move backwards instead
            if distance < 0 and abs(angle) > 80:
                move(robot=robot, distance=reverse_distance)

            # Turn the robot right
            turn(robot=robot, angle=angle)

    elif instruction["instruction"] == "Forward":
        distance = float(instruction["distance"])
        angle = float(instruction["angle"])

        print("Distance to wall:", distance_to_wall)
        print("Distance to ball:", distance)

        # If the distance to the wall is less than the stop distance or more than 2000, move in reverse
        if distance_to_wall <= stop_distance or distance_to_wall > 2000:
            wait(500)  # Wait for 1 second
            robot.straight(reverse_distance)

        # If the distance to the wall is somewhat the same at the distance to the ball, move half the distance
        elif abs(distance_to_wall - (distance * 10)) <= 50:
            print("Wall and ball are within similar range. Moving half of the distance")
            move(robot=robot, distance=distance / 2)
            wait(500)  # Wait for 0.5 seconds

        # If the distance to the wall is less than or equal to 100 cm, move a different distance
        elif distance_to_wall <= 300:
            print("Moving half of distance to the wall")
            move(robot=robot, distance=distance - 1)  # Move 2 cm less than the distance
            wait(500)

        # If none of the above conditions are met, perform these steps
        else:
            # If the distance is under 15 and absolute angle is over 80, move backwards instead
            if distance < 0 and abs(angle) > 80:
                move(robot=robot, distance=reverse_distance)
                wait(500)  # Wait for 0.5 seconds

            # If the distance to the ball is over 35, move half the distance
            elif distance > 35:
                move(robot=robot, distance=distance / 2)
                wait(500)  # Wait for 0.5 seconds
            else:
                move(robot=robot, distance=distance)
                wait(500)  # Wait for 0.5 seconds

    elif instruction["instruction"] == "Shoot":
        four_wheel_mechanism_thread.stop()  # Stop the four_wheel_mechanism thread
        # Start the outward spinner thread
        spinner_thread = SpinnerThreadOutward(spinner=spinner)
        spinner_thread.start()
        
        # Release the four_wheel_mechanism
        release_four_wheel_mechanism(four_wheel_mechanism=four_wheel_mechanism) 

        global stopInstructions
        stopInstructions = True


def turn(robot: DriveBase, angle):
    # Code to turn the robot left
    print("Turning:", angle)
    # Adjust the motor rotation and speed according to your robot's setup
    robot.turn(angle=angle)


def move(robot: DriveBase, distance):
    # Code to move the robot forward
    print("Moving forward:", distance)
    # Adjust the motor rotation and speed according to your robot's setup
    robot.straight(distance=distance * 10)  # Convert to millimeters


def stop(robot: DriveBase):
    robot.stop()


def check_for_red(sensor: ColorSensor, robot: DriveBase):
    if sensor.color() == Color.RED:
        stop(robot=robot)


# Run conveyor function to be started on a separate thread
def run_conveyor(conveyor: Motor):
    print("Running conveyor")
    conveyor.run_time(speed=1000, time=10000 * 48)  # Run the conveyor belt for 10 seconds * 48 = 480 seconds


def release_four_wheel_mechanism(four_wheel_mechanism: Motor):
    print("Releasing balls")
    four_wheel_mechanism.run_time(speed=-1000, time=10000)  # Run the four wheel mechanism for 10 seconds


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