# GolfBot - EV3 Robot
This project demonstrates a LEGO EV3 robot that can pick up balls and navigate on a field based on instructions received from a server that is connected to a computer vision program that detect the balls in real-time using OpenCV.

<img width="1600" alt="image" src="https://github.com/yukarasan/GolfBot/assets/91070526/f2f4e604-54e3-42f7-8ecc-f5897b9be6ac">

**^^**
[Watch the video](https://www.youtube.com/watch?v=U4peSkA70Z4&ab_channel=MathildeElia)

## Features (as of 21-06-2023)

1. LEGO EV3 robot with a four-wheel mechanism to pick up balls, and a fork mechanism to push balls into the picker. 
2. A server that sends instructions to the robot, such as `"Forward"`, `"Left"`, `"Right"` or `"Shoot"`, with their associated data such as `"angle"`, `"distance"` and more.
3. A Client that can request instructions to the server, and use the recieved data to perform different operations depending on the recieved data. 
4. Real-time ball detection using OpenCV.
   * Orange **(FEATURE REMOVED)** and white table tennis balls.
5. Direction and position of robot using angle calculations.
6. Direction and position of balls relative to the robot using angle calculations and distance calculation.
7. Finding the ball closest to the robot.
8. Direction and position of goals using angle calculations and distance calculation.
9. Aligning the position of the robot relative to objects positions. Examples: 
   * Going to the goal, the robot alligns itself in front of the goal
   * Going after a ball close to the wall or corner, the robot alligns itself in front of the ball witin a given angle.
10. A conversion factor which is calculated to convert pixel distances to real-world distances (in centimeters).
    * Based on known size of the field.
11. Avoiding walls by using a ultrasonic sensor that sends sound waves to determine the distance to a wall. **(FEATURE REMOVED)**
    * This is achived by making the robot move different distances depending on the distance to the wall.  
12. Avoiding the detection of balls outside the borders of the course.
13. When scoring, the robot will play a winning sound of victory!

## Requirements

- LEGO MINDSTORMS EV3 running MicroPython v2.0 or higher.
- Python 3.x.
- OpenCV 4.x.
- Numpy.
- Math.
- Thread.
- time.
- Flask (server).
- jsonfy (server).
- socket (client).
- Enum (used to determine instructions).
- A working camera connected to the computer/program running the "server".
   * For Windows, remember to change the code: `cap = cv2.VideoCapture(0)` to `cap = cv2.VideoCapture(1)`.

## Getting Started

1. Clone this repository.
2. Set up your LEGO EV3 robot with the necessary motors and sensors. 
    * Left wheel (motor port.A)
    * Right wheel (Motor port.B)
    * Spinner mechanism (Motor port.C)
    * Four-wheel mechanism (Motor port.D)
    * Touch sensor (Port.S3)
4. Run the server code on your computer, to get the IP address and port number. 
5. Run the robot code on your EV3 brick by using an IDE such as Visual Studio Code.
