import string
from enum import Enum


class Instructions(Enum):
    MOVE_FORWARD = "Foward"
    MOVE_BACKWARD = "Backward"
    MOVE_RIGHT = "Right"
    MOVE_LEFT = "Left"


def determineNextMove(angle_of_robot: float, angle_to_destination: float, distance_to_destination: float):
    instruction = ""
    if angle_to_destination < angle_of_robot:
        instruction = Instructions.MOVE_RIGHT.value + " " + determineAngleToMove(angle_of_robot, angle_to_destination)

    return instruction


def determineAngleToMove(angle_of_robot: float, angle_to_destination: float):
    return "" + angle_of_robot + angle_to_destination


if __name__ == "__main__":
    print(determineNextMove(1.0, 0.0, 10.0))
