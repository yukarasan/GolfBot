# GolfBot - EV3 Robot
This project demonstrates a LEGO EV3 robot that can pick up balls and navigate on a field based on instructions received from a server that is connected to a computer vision program that detect the balls in real-time using OpenCV.

## Features (as of 26-04-2023)

1. LEGO EV3 robot with a conveyor belt mechanism to pick up balls.
2. A server that sends instructions to the robot, such as "pick_balls" or "drive_straight".
3. Real-time ball detection using OpenCV.

## Requirements

- LEGO MINDSTORMS EV3 running MicroPython v2.0 or higher.
- Python 3.x.
- OpenCV 4.x.
- Numpy.

## Getting Started

1. Clone this repository.
2. Set up your LEGO EV3 robot with the necessary motors and sensors. 
    * Left wheel (motor port C)
    * Right wheel (Motor port B)
    * Conveyor belt (Motor port D)
3. Update the IP address and port number in the robot code to match your server settings.
4. Run the server code on your computer.
5. Transfer and run the robot code on your EV3 brick by using an IDE such as Visual Studio Code.

## Usage

The robot will receive instructions from the server and execute them. It can perform the following tasks:

- Picking up balls: ??????????????????
- Driving straight: The robot will simply drive straight for a specified distance.
