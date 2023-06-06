# GolfBot - EV3 Robot
This project demonstrates a LEGO EV3 robot that can pick up balls and navigate on a field based on instructions received from a server that is connected to a computer vision program that detect the balls in real-time using OpenCV.

![image](https://github.com/yukarasan/GolfBot/assets/91070526/cdbf89ed-270c-4629-abbc-d01c4f72340b)

## Features (as of 06-06-2023)

1. LEGO EV3 robot with a conveyor belt mechanism to pick up balls.
2. A server that sends instructions to the robot, such as "pick_balls" or "drive_straight".
3. Real-time ball detection using OpenCV.
   * Orange and white table tennis balls.
4. Direction and position of robot using angle calculations.
5. Direction and position of balls using angle calculations and distance calculation.
6. Finding the ball closest to the robot.
7. Direction and position of goals using angle calculations and distance calculation.
8. A conversion factor which is calculated to convert pixel distances to real-world distances (in centimeters).
   * Based on known size of the field
9. ??????????????????

## Requirements

- LEGO MINDSTORMS EV3 running MicroPython v2.0 or higher.
- Python 3.x.
- OpenCV 4.x.
- Numpy.
- Math
- Flask (server)
- requests (client) 
- Enum (used to determine instructions)
- A working camera connected to the computer/program running the "server"
   * For Windows, remember to change the code to cap = cv2.VideoCapture(0) to cap = cv2.VideoCapture(1)

## Getting Started

1. Clone this repository.
2. Set up your LEGO EV3 robot with the necessary motors and sensors. 
    * Left wheel (motor port C)
    * Right wheel (Motor port B)
    * Conveyor belt (Motor port D)
4. Run the server code on your computer, to get the IP address and port number. 
5. Run the robot code on your EV3 brick by using an IDE such as Visual Studio Code.

## Usage

The robot will receive instructions from the server and execute them. It can perform the following tasks:

- Picking up balls: ??????????????????
- Driving straight: The robot will simply drive straight for a specified distance.
