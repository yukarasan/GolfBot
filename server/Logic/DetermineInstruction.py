from enum import Enum
from flask import Flask, jsonify
app=Flask(__name__)

class Instructions(Enum):
    MOVE_FORWARD = "Forward"
    MOVE_BACKWARD = "Backward"
    MOVE_RIGHT = "Right"
    MOVE_LEFT = "Left"
    SHOOT = "Shoot"
    FORWARD_THEN_SHOOT = "Forward_Shoot"

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

def determine_turn_direction(angle1, angle2, distance):
    shortest_angle = calculate_shortest_angle(angle1, angle2)

    ##if vi er tæt på, så må vinklen godt være større
    ##if vi er meget tæt på, og vinklen er stor, så go backwards
    if distance < 25 and abs(shortest_angle) <= 2.9:
        return Instructions.MOVE_FORWARD.value
    elif abs(shortest_angle) <= 2.5:
        return Instructions.MOVE_FORWARD.value
    if shortest_angle < 0:
        return Instructions.MOVE_RIGHT.value
    elif shortest_angle > 0:
        return Instructions.MOVE_LEFT.value

#
def determine_goal_instruction(angle1, angle2, distance_to_goal, distance_to_goal_point, angle_to_goal_point):

    #angle mellem bold og robot
    shortest_angle = calculate_shortest_angle(angle1, angle2)

    #Hvornår robotten skal skyde
    if (distance_to_goal <= 16 and abs(shortest_angle) <= 12) or (distance_to_goal <= 14 and abs(shortest_angle) <= 23):
        return (Instructions.SHOOT.value, 4.00, 0.00)

    #Deciding when to go to the goal point
    elif distance_to_goal_point >= 15 and abs(shortest_angle) >= 63:
        return (determine_turn_direction(angle_to_goal_point, angle2, distance_to_goal_point), calculate_shortest_angle(angle2, angle_to_goal_point), distance_to_goal_point)

    #Turn and drive towards the goal
    else:
        return (determine_turn_direction(angle1, angle2, distance_to_goal), calculate_shortest_angle(angle2, angle1), distance_to_goal - 6)
