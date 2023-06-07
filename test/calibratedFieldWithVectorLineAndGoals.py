import cv2
import numpy as np
import math
from flask import Flask, jsonify
import threading

from server.Logic.DetermineInstruction import Instructions, determineAngleToMove, determine_turn_direction, \
    calculate_shortest_angle

app = Flask(__name__)

def flask_server():
    app.run(port=8081)



@app.route("/")
def determineNextMove():
    data = {"instruction": determine_turn_direction(angle_to_destination, angle_of_robot),
            "angle": "{:.2f}".format(calculate_shortest_angle(angle_of_robot, angle_to_destination)),
            "distance": "{:.2f}".format(goal_distance)}
    return jsonify(data)

angle_to_destination = 0
angle_of_robot = 0
goal_distance = 0

def flask_server():
    app.run(host = "0.0.0.0", port=8081)

cap = cv2.VideoCapture(0)

# Range for green
lower_green = np.array([30, 30, 30], dtype=np.uint8)
upper_green = np.array([80, 255, 255], dtype=np.uint8)

# Range for white
lower_white = np.array([0, 0, 200])
upper_white = np.array([180, 30, 255])

# Range for red
red_lower = np.array([0, 100, 100])
red_upper = np.array([10, 255, 255])

# Define lower and upper bounds for orange color
lower_orange = np.array([20, 100, 100])
upper_orange = np.array([30, 255, 255])

kernel = np.ones((5, 5), np.uint8)


def calculate_angle_between_lines(p1, p2, p3, p4):
    # Calculate slopes

    if p2[1] - p1[1] != 0 and p2[0] - p1[0] != 0 and p4[1] - p3[1] != 0 and p4[0] - p3[0] != 0:
        m1 = (p2[1] - p1[1]) / (p2[0] - p1[0])
        m2 = (p4[1] - p3[1]) / (p4[0] - p3[0])
    else:
        m1 = 0
        m2 = 0

    # Calculate angle
    if 1 + m1 * m2 != 0 and m2 - m1 != 0:
        tan_theta = abs((m2 - m1) / (1 + m1 * m2))
        angle = math.degrees(math.atan(tan_theta))
    else:
        angle = 0

    # Calculate cross product
    v1 = [p2[0] - p1[0], p2[1] - p1[1]]  # vector 1, green line
    v2 = [p4[0] - p3[0], p4[1] - p3[1]]  # vector 2, red line
    cross_product = v1[0] * v2[1] - v1[1] * v2[0]

    if cross_product < 0:
        angle = -angle

    return angle


def calculate_distance(pt1, pt2):
    # Calculate Euclidean distance between two points
    return np.sqrt((pt1[0] - pt2[0]) ** 2 + (pt1[1] - pt2[1]) ** 2)


def calculate_angle(center1, center2):
    x1, y1 = center1
    x2, y2 = center2
    angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
    return angle


def draw_line(image, start, end, color, thickness=2):
    height, width = image.shape[:2]

    # Calculate the line endpoints that extend to the edges of the frame
    x1, y1 = start
    x2, y2 = end

    if x1 == x2:  # Vertical line
        x1 = x2 = max(0, min(x1, width - 1))
        y1 = 0
        y2 = height - 1
    else:
        m = (y2 - y1) / (x2 - x1)  # Slope
        b = y1 - m * x1  # Intercept

        x1 = 0
        y1 = int(b)
        x2 = width - 1
        y2 = int(m * x2 + b)

    cv2.line(image, (x1, y1), (x2, y2), color, thickness)


def draw_line_to_goals(image, start, end, color, thickness=2):
    # Validate the image input
    if len(image.shape) < 2:
        raise ValueError("Invalid image input")

    # Draw a line from start to end using cv2.line() function
    cv2.line(image, start, end, color, thickness)


# Initialize conversion factor
conversion_factor = None

# Create and start the Flask server in a separate thread
flask_thread = threading.Thread(target=flask_server)
flask_thread.start()

while True:
    ret, frame = cap.read()

    if not ret:
        break

    wall_thickness = 550

    frame_height, frame_width = frame.shape[:2]

    # Calculate the midpoints for the goals
    # Adjusting them by half of the wall's thickness
    goal_left = (wall_thickness // 2, frame_height // 2)  # Left side goal
    goal_right = (frame_width - 1 - wall_thickness // 2, frame_height // 2)  # Right side goal

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Mask for green object
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    # Mask for white object
    mask_white = cv2.inRange(hsv, lower_white, upper_white)

    # Mask for red object
    mask_red = cv2.inRange(hsv, red_lower, red_upper)

    # Define lower and upper bounds for blue color
    lower_pink = np.array([150, 50, 50])
    upper_pink = np.array([180, 255, 255])

    # Define lower and upper bounds for green color
    lower_green = np.array([40, 50, 50])
    upper_green = np.array([80, 255, 255])
    green_center = (int(0), int(0))

    # Threshold the image to get only blue and green regions
    blue_mask = cv2.inRange(hsv, lower_pink, upper_pink)
    green_mask = cv2.inRange(hsv, lower_green, upper_green)

    mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)

    # Find contours for blue and green regions
    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Find the largest blue and green contours
    pink_contour = max(blue_contours, key=cv2.contourArea) if blue_contours else None
    green_contour = max(green_contours, key=cv2.contourArea) if green_contours else None

    if pink_contour is not None and green_contour is not None:
        # Get the center points of blue and green rectangles
        pink_moment = cv2.moments(pink_contour)
        green_moment = cv2.moments(green_contour)

        if pink_moment["m00"] != 0 and green_moment["m00"] != 0:
            pink_center = (int(pink_moment["m10"] / pink_moment["m00"]), int(pink_moment["m01"] / pink_moment["m00"]))
            green_center = (int(green_moment["m10"] / green_moment["m00"]), int(green_moment["m01"] / green_moment["m00"]))

            # Calculate the angle between the centers of blue and green rectangles
            angle_of_robot = robot_angle = calculate_angle(pink_center, green_center)

            # Draw a line connecting the centers of blue and green rectangles
            draw_line(frame, pink_center, green_center, (0, 255, 0), thickness=2)

            # Display the angle on the frame
            cv2.putText(frame, "Robot angle: {:.2f}".format(robot_angle), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 0), 2)


    # Apply morphological operations
    mask_green = cv2.erode(mask_green, kernel, iterations=1)
    mask_green = cv2.dilate(mask_green, kernel, iterations=1)
    mask_white = cv2.erode(mask_white, kernel, iterations=1)
    mask_white = cv2.dilate(mask_white, kernel, iterations=1)
    mask_red = cv2.erode(mask_red, kernel, iterations=1)
    mask_red = cv2.dilate(mask_red, kernel, iterations=1)
    mask_orange = cv2.erode(mask_orange, kernel, iterations=1)
    mask_orange = cv2.dilate(mask_orange, kernel, iterations=1)

    # Blur mask for red object
    blur_red = cv2.blur(mask_red, (14, 14))

    # Find contours for red object
    contours_red, _ = cv2.findContours(blur_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(frame, contours_red, -1, (0, 255, 0), 3)

    # Calculate conversion factor
    if conversion_factor is None and contours_red:
        # Get the largest red contour
        contours_red = sorted(contours_red, key=cv2.contourArea, reverse=True)
        red_contour = contours_red[0]

        # Get rotated rectangle
        rect = cv2.minAreaRect(red_contour)
        box = cv2.boxPoints(rect)
        box = np.intp(box)

        # Get length and width of the rectangle
        width_pixels = int(rect[1][0])
        height_pixels = int(rect[1][1])

        # Calculate conversion factors
        conversion_factor_length = 180 / max(width_pixels, height_pixels)  # Considering length as the max dimension
        conversion_factor_width = 120 / min(width_pixels, height_pixels)  # Considering width as the min dimension

        # Take average of both conversion factors
        conversion_factor = (conversion_factor_length + conversion_factor_width) / 2

    # Find green object centroid
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    contours_green = sorted(contours_green, key=cv2.contourArea, reverse=True)
    contours_orange, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # cv2.imshow('orange count', mask_orange)

    if contours_green:
        M = cv2.moments(contours_green[0])
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)
        cv2.putText(frame, f"centroid {cX}, {cY}", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Find white object and draw minimum enclosing circle
        contours_white, _ = cv2.findContours(mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_distance = np.inf
        min_distance_orange = np.inf
        closest_ball_center = None

        for cnt in contours_white:
            if cnt.shape[0] > 5:
                (x, y), radius = cv2.minEnclosingCircle(cnt)
                center = (int(x), int(y))
                radius = int(radius)
                if radius > 11 and radius < 20:
                    cv2.circle(frame, center, radius, (0, 255, 0), 2)
                    cv2.putText(frame, f"ball {center[0]}, {center[1]}", (center[0] - 20, center[1] - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # Calculate distance between centroid of green object and center of white ball
                    pixel_distance = calculate_distance(center, (cX, cY))
                    pixel_distance = calculate_distance(center, (cX, cY))

                    if pixel_distance < min_distance:
                        min_distance = pixel_distance
                        closest_ball_center = center

        for cnt in contours_orange:
            # Fit a circle to the contour if it has enough points
            if cnt.shape[0] > 5:
                (x, y), radius = cv2.minEnclosingCircle(cnt)
                center = (int(x), int(y))
                radius = int(radius)

                # Draw the circle if it's big enough and track it
                if radius > 10 and radius < 20:
                    cv2.circle(frame, center, radius, (0, 165, 255), 2)  # use orange color for orange circle
                    prevOrangeCircle = center + (radius,)
                    # Orange ball:
                    cv2.circle(frame, center, radius, (0, 255, 255), 2)  # Draw the circle with a cyan color
                    cv2.putText(frame, f"orange ball {center[0]}, {center[1]}", (center[0] - 20, center[1] - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

                    # Calculate distance between centroid of green object and center of orange ball
                    pixel_distance = calculate_distance(center, (cX, cY))
                    pixel_distance = calculate_distance(center, (cX, cY))

                    if pixel_distance < min_distance:
                        min_distance = pixel_distance
                        closest_ball_center = center

                    # Calculate distance between centroid of green object and center of orange ball
                    pixel_distance = calculate_distance(center, (cX, cY))
                    if pixel_distance < min_distance_orange:
                        min_distance_orange = pixel_distance
                        closest_orange_ball_center = center

        if closest_ball_center is not None:
            # Draw a line between centroid of green object and center of the closest white ball
            cv2.line(frame, (cX, cY), closest_ball_center, (0, 0, 255), 2)

            # Convert pixel distance to cm and display it on the frame

            goal_distance = distance_cm = min_distance * conversion_factor
            cv2.putText(frame, f"Distance: {distance_cm:.2f} cm", (cX - 20, cY - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 0, 255), 2)

            # Calculate and display angle between the two lines
            angle_to_destination = ball_angle = calculate_angle(green_center, closest_ball_center)


            cv2.putText(frame, f"Angle to ball: {ball_angle:.2f}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (255, 0, 0), 2)


        ###############################################################################################################
        ########################################### FINDING DISTANCE TO GOAL ##########################################
        ###############################################################################################################

        # Draw the goals on the frame
        cv2.circle(frame, goal_left, radius=8, color=(0, 255, 255), thickness=-2)  # Yellow dot
        cv2.circle(frame, goal_right, radius=8, color=(0, 255, 255), thickness=-2)  # Yellow dot

        # Calculate distances to the goals
        distance_to_left_goal = calculate_distance(green_center, goal_left)
        distance_to_right_goal = calculate_distance(green_center, goal_right)

        goal_angle = None
        # Draw a line to the closest goal
        if distance_to_left_goal < distance_to_right_goal:
            draw_line_to_goals(frame, green_center, goal_left, (0, 255, 255), thickness=2)
            goal_angle = calculate_angle(green_center, goal_left)
        else:
            goal_angle = calculate_angle(green_center, goal_right)
            draw_line_to_goals(frame, green_center, goal_right, (0, 255, 255), thickness=2)

        cv2.putText(frame, f"Angle to goal: {goal_angle:.2f}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (255, 0, 0), 2)


    cv2.imshow('All Contours', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        flask_thread.join()
        break

cap.release()
cv2.destroyAllWindows()
