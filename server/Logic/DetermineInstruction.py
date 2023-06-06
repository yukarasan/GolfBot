from enum import Enum
from flask import Flask, jsonify
app=Flask(__name__)

class Instructions(Enum):
    MOVE_FORWARD = "Foward"
    MOVE_BACKWARD = "Backward"
    MOVE_RIGHT = "Right"
    MOVE_LEFT = "Left"

def calculate_shortest_angle(angle1, angle2):
    # Normalize the angles to the range -180 to 180
    angle1 = angle1 % 360
    angle2 = angle2 % 360

    # Calculate the difference between the angles
    diff = angle2 - angle1

    # Adjust the difference to the range -180 to 180
    if diff > 180:
        diff -= 360
    elif diff < -180:
        diff += 360

    return diff

def determine_turn_direction(angle1, angle2):
    shortest_angle = calculate_shortest_angle(angle1, angle2)

    if abs(shortest_angle) <= 5:
        return Instructions.MOVE_FORWARD
    if shortest_angle < 0:
        return Instructions.MOVE_RIGHT.value
    elif shortest_angle > 0:
        return Instructions.MOVE_LEFT.value


def determineAngleToMove(angle_of_robot: float, angle_to_destination: float):

    if angle_of_robot <= 0 and angle_to_destination >= 0 :
        return abs(angle_of_robot) + abs(angle_to_destination)

    return abs(abs(angle_of_robot) - abs(angle_to_destination))