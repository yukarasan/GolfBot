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
# Global variable to keep track of whether the robot has reached the end
stopInstructions = False
# Global variable to keep track of whether the robot is going to the goal
going_to_goal = 0

# Global variable to count the number of times the touch sensor has been pressed
press_count = 0
last_press_state = False

# Global variable to keep track of the second round
second_round = False


# Thread for the four_wheel_mechanism belt
class FourWheelMechanism(Thread):
    def __init__(self, four_wheel_mechanism):
        Thread.__init__(self)
        self.four_wheel_mechanism = four_wheel_mechanism
        self.running = True

    def run(self):
        while self.running:
            self.four_wheel_mechanism.run(1000)  # Run the four_wheel_mechanism motor continuously
            time.sleep(0.1)  # Sleep for a short while to not hog the CPU

    def stop(self):
        self.running = False
        self.four_wheel_mechanism.stop()  # Stops the four_wheel_mechanism motor

    def stop_four_wheel_mechanism(self):
        self.four_wheel_mechanism.stop()  # Stops the four_wheel_mechanism motor


# Thread for the spinner that spins inwards
class SpinnerThreadInwards(Thread):
    def __init__(self, spinner):
        Thread.__init__(self)
        self.spinner = spinner
        self.running = True

    # Make the spinner run continuously
    def run(self):
        while self.running:
            self.spinner.run(speed=-300)

            # Stops the spinner motor

    def stop(self):
        self.running = False
        self.spinner.stop()

    # Thread for the spinner that spins outwards


class SpinnerThreadOutward(Thread):
    def __init__(self, spinner):
        Thread.__init__(self)
        self.spinner = spinner
        self.running = True

    # Make the spinner run continuously, but in the opposite direction
    def run(self):
        while self.running:
            self.spinner.run(speed=300)

    def stop(self):
        self.running = False
        self.spinner.stop()


# This program requires LEGO EV3 MicroPython v2.0 or higher
def main():
    # Objects and setup
    ev3 = EV3Brick()
    left_wheel = Motor(Port.A)
    right_wheel = Motor(Port.B)
    four_wheel_mechanism = Motor(Port.D)
    spinner = Motor(Port.C)
    left_touch_sensor = TouchSensor(Port.S3)
    # right_touch_sensor = TouchSensor(Port.S2)

    # Wheel diameter and axle track (in millimeters)
    wheel_diameter = 56
    axle_track = 30

    # DriveBase object
    robot = DriveBase(left_wheel, right_wheel, wheel_diameter, axle_track)

    # Soundfile object to play the winning sound when the robot reaches the end
    winning_sound = SoundFile.FANFARE
    # ImageFile object to display the winning image when the robot reaches the end
    winning_image = ImageFile.THUMBS_UP

    # Make the robot move forward for 10 seconds and after that reverse for 5 seconds
    remove_cross(robot=robot)

    # Start the four_wheel_mechanism belt thread
    four_wheel_mechanism_thread = FourWheelMechanism(four_wheel_mechanism)
    four_wheel_mechanism_thread.start()

    # Start the spinner threads
    spinner_thread = SpinnerThreadInwards(spinner)
    spinner_thread.start()
    outwards_spinner_thread = SpinnerThreadOutward(spinner)

    global second_round
    global press_count
    global going_to_goal

    # Make the robot ask for instructions until it stopInstructions is True
    while stopInstructions is not True:

        if second_round:
            # Start the four_wheel_mechanism belt thread
            four_wheel_mechanism_thread = FourWheelMechanism(four_wheel_mechanism)
            four_wheel_mechanism_thread.start()

            # Start the spinner threads
            spinner_thread = SpinnerThreadInwards(spinner)
            spinner_thread.start()
            outwards_spinner_thread = SpinnerThreadOutward(spinner)
            outwards_spinner_thread.start()

            going_to_goal = 0

        # Socket connection setup
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(("172.20.10.4", 8081))  # 10.209.234.177 || 172.20.10.4

        check_press(left_touch_sensor)

        print("Press count:", press_count)

        reverse_distance = -100  # Distance to move backwards (10 cm)

        # If an object is detected within the stop distance, stop and move backwards else keep asking for instructions
        if (press_count >= 1) and going_to_goal == 0:
            print("The press count has been reached")
            press_count = 0
            # wait(500)  # Wait for 500 milliseconds
            robot.straight(reverse_distance)
        else:
            try:
                # Send request
                request = "GET / HTTP/1.1\r\nHost: 172.20.10.4\r\n\r\n"
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
                    # Process instruction based on JSON data
                    process_instruction(
                        robot=robot,
                        instruction=json_data,
                        four_wheel_mechanism=four_wheel_mechanism,
                        four_wheel_mechanism_thread=four_wheel_mechanism_thread,
                        spinner_thread=spinner_thread,
                        outwards_spinner_thread=outwards_spinner_thread,
                        reverse_distance=reverse_distance
                    )
                else:
                    print('Request failed.')

            except Exception as e:
                print(e)

            finally:
                sock.close()

    if outwards_spinner_thread.running:
        outwards_spinner_thread.stop()

    # Stop the four_wheel_mechanism belt thread
    four_wheel_mechanism.stop()

    ev3.speaker.play_file(winning_sound)  # Play the winning sound
    ev3.screen.load_image(winning_image)  # Display the winning image
    # wait(2000)  # Wait for 5 seconds


# Function to process the instruction from the server and move the robot accordingly
def process_instruction(
        robot: DriveBase,
        instruction,
        four_wheel_mechanism: Motor,
        four_wheel_mechanism_thread: FourWheelMechanism,
        spinner_thread: SpinnerThreadInwards,
        outwards_spinner_thread: SpinnerThreadOutward,
        reverse_distance: int
):
    global press_count

    # if instruction "go to goal" is "yes" then stop the spinner
    if instruction["go to goal"] == "yes":
        global going_to_goal
        going_to_goal += 1
        if going_to_goal == 3:
            spinner_thread.stop()
    else:
        going_to_goal = 0

    if instruction["instruction"] in ["Left", "Right"]:
        global direction_counter
        direction_counter += 1

        # Printing the amount of times the robot has turned left or right
        print("Direction counter:", direction_counter)

        # If the robot has turned left or right 20 times, make the robot move backwards if and only
        # if it is not going to the goal and is not hitting the wall
        if direction_counter >= 15:
            direction_counter = 0

            if (press_count >= 1) and going_to_goal == 0:
                print("The press count has been reached")
                press_count = 0
                # wait(500)
                robot.straight(reverse_distance)
            else:
                move(robot=robot, distance=-10)  # move backwards 10 cm
    else:
        direction_counter = 0  # Reset the counter if the instruction is not "Left" or "Right"

    if instruction["instruction"] == "Left":
        angle = float(instruction["angle"])

        if (press_count >= 1) and going_to_goal == 0:
            print("The press count has been reached")
            press_count = 0
            # wait(500)
            robot.straight(reverse_distance)
        else:
            # Turn the robot left
            turn(robot=robot, angle=angle)

    elif instruction["instruction"] == "Right":
        angle = float(instruction["angle"])

        if (press_count >= 1) and going_to_goal == 0:
            print("The press count has been reached")
            press_count = 0
            # wait(500)
            robot.straight(reverse_distance)
        else:
            # Turn the robot right
            turn(robot=robot, angle=angle)

    elif instruction["instruction"] == "Forward":
        distance = float(instruction["distance"])

        if (press_count >= 1) and going_to_goal == 0:
            print("The press count has been reached")
            press_count = 0
            wait(500)
            robot.straight(reverse_distance)
        else:
            # If the distance to the ball is over 35, move half the distance
            if distance > 35:
                move(robot=robot, distance=distance / 2)
                wait(500)
            else:
                move(robot=robot, distance=distance)
                wait(500)

    elif instruction["instruction"] == "Shoot":
        four_wheel_mechanism_thread.stop()  # Stop the four_wheel_mechanism thread

        # Stop the inwards spinner thread
        spinner_thread.stop()
        # Start the outward spinner thread
        outwards_spinner_thread.start()

        wait(500)  # Wait for 500 milliseconds

        # Release the four_wheel_mechanism
        release_four_wheel_mechanism(four_wheel_mechanism=four_wheel_mechanism)

        global second_round
        second_round = True

        # Make the robot move backwards for 10 cm for the second round
        move(robot=robot, distance=-10)

        #global stopInstructions
        #stopInstructions = True


# A function to turn the robot left or right depending on the angle
def turn(robot: DriveBase, angle):
    robot.turn(angle=angle)


# A function to move the robot forward or backward depending on the distance
def move(robot: DriveBase, distance):
    robot.straight(distance=distance * 10)  # Converting to millimeters


# A function to stop the robot
def stop(robot: DriveBase):
    robot.stop()


# A function to check if the touch sensor is pressed and increment the press count
def check_press(touch_sensor: TouchSensor):
    global press_count
    global last_press_state

    if going_to_goal == 0:
        current_state = touch_sensor.pressed()
        
        if current_state and not last_press_state:  # if the sensor is pressed now and wasn't pressed in the last check
            press_count += 1
            
        last_press_state = current_state
    else: 
        last_press_state = False


# A function that releases the four wheel mechanism to shoot the balls in the goal
def release_four_wheel_mechanism(four_wheel_mechanism: Motor):
    print("Releasing balls")
    four_wheel_mechanism.run_time(speed=-1000, time=10000)  # Run the four wheel mechanism for 10 seconds


# Play winning sound when the robot finishes the course
def play_winning_sound(ev3: EV3Brick, soundfile):
    try:
        ev3.speaker.set_volume(100)
        ev3.speaker.play_file(soundfile)
    except Exception as e:
        print("Failed to play sound.")


def remove_cross(robot: DriveBase): 
    robot.drive(100, 0)
    wait(11000)
    robot.drive(-100, 0)
    wait(5000)
    robot.stop()


# Display winning image on the EV3 screen when the robot finishes the course
def display_winning_image(ev3: EV3Brick, imagefile):
    try:
        ev3.screen.load_image(imagefile)
    except Exception as e:
        print("Failed to display image.")


if __name__ == "__main__":
    main()