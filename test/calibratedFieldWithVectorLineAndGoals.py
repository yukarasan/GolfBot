import cv2
import numpy as np
import math
from flask import Flask, jsonify
import threading

from server.Logic.DetermineInstruction import determine_turn_direction, \
    calculate_shortest_angle, determine_goal_instruction

app = Flask(__name__)

def flask_server():
    app.run(port=8081)


@app.route("/")
def determineNextMove():
    # if nuværende antalBolde == 5 || antal == 0 --> gå til goal, else --> gå til nærmeste bold
    #if num_balls_white + num_balls_orange == 5 or num_balls_white + num_balls_orange == 0:
    if num_balls == 0:
        goal_instruction = determine_goal_instruction(angle_to_goal, angle_of_robot, goal_distance, distance_to_goal_point=goal_point_distance, angle_to_goal_point=goal_point_angle)
        data = {"instruction": goal_instruction[0],
                "angle": "{:.2f}".format(goal_instruction[1]),
                "distance": "{:.2f}".format(goal_instruction[2]),
                "go to goal": "yes"
                }
    else:
        data = {"instruction": determine_turn_direction(angle_to_ball, angle_of_robot, ball_distance),
                "angle": "{:.2f}".format(calculate_shortest_angle(angle_of_robot, angle_to_ball)),
                "distance": "{:.2f}".format(ball_distance),
                "go to goal": "no"
                }

    return jsonify(data)


angle_to_ball = 0
angle_of_robot = 0
ball_distance = 0

angle_to_goal = 0
goal_distance = 0

goal_point_distance = 0
goal_point_angle = 0

num_balls = None


def flask_server():
    app.run(host="0.0.0.0", port=8081)


cap = cv2.VideoCapture(0)

# Range for green
lower_green = np.array([30, 30, 30], dtype=np.uint8)
upper_green = np.array([80, 255, 255], dtype=np.uint8)

# Range for white
lower_white = np.array([0, 0, 200])
upper_white = np.array([180, 30, 255])

#Range for red
red_lower = np.array([0, 100, 100])
red_upper = np.array([10, 255, 255])
#rgb_color = np.uint8([[[199, 54, 52 ]]]) # given RGB color
#hsv_color = cv2.cvtColor(rgb_color, cv2.COLOR_RGB2HSV)

# assuming hsv_color is the color converted into HSV
#h, s, v = hsv_color[0][0]
#red_lower = np.array([h - 10, 100, 100])
#red_upper = np.array([h + 10, 255, 255])

# Define lower and upper bounds for orange color
lower_orange = np.array([25, 50, 20])
upper_orange = np.array([32, 110, 255])

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

def calculate_new_coordinates(center, angle, distance):
    x, y = center
    angle_radians = math.radians(angle)
    new_x = x - distance * math.cos(angle_radians)
    new_y = y - distance * math.sin(angle_radians)
    return int(new_x), int(new_y)


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

    goal_point_left = (1200 // 2, frame_height // 2)  # Left side goal
    goal_point_right = (frame_width - 1 - 1200 // 2, frame_height // 2)  # Right side goal

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
    pink_mask = cv2.inRange(hsv, lower_pink, upper_pink)
    green_mask = cv2.inRange(hsv, lower_green, upper_green)

    mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)

    # Find contours for blue and green regions
    pink_contours, _ = cv2.findContours(pink_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Find the largest blue and green contours
    pink_contour = max(pink_contours, key=cv2.contourArea) if pink_contours else None
    green_contour = max(green_contours, key=cv2.contourArea) if green_contours else None

    if pink_contour is not None and green_contour is not None:
        # Get the center points of blue and green rectangles
        pink_moment = cv2.moments(pink_contour)
        green_moment = cv2.moments(green_contour)

        if pink_moment["m00"] != 0 and green_moment["m00"] != 0:
            pink_center = (int(pink_moment["m10"] / pink_moment["m00"]), int(pink_moment["m01"] / pink_moment["m00"]))
            green_center = (
            int(green_moment["m10"] / green_moment["m00"]), int(green_moment["m01"] / green_moment["m00"]))

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

    #Blur mask for red object
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
        conversion_factor_length = 182 / max(width_pixels, height_pixels)  # Considering length as the max dimension
        conversion_factor_width = 132 / min(width_pixels, height_pixels)  # Considering width as the min dimension

        # Take average of both conversion factors
        conversion_factor = (conversion_factor_length + conversion_factor_width) / 2

    # Find green object centroid
    contours_pink, _ = cv2.findContours(pink_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    contours_pink = sorted(contours_pink, key=cv2.contourArea, reverse=True)
    contours_orange, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #cv2.imshow('orange count', mask_red)

    if contours_pink:
        M = cv2.moments(contours_pink[0])
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)
        cv2.putText(frame, f"centroid {cX}, {cY}", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255),
                    2)

        # Find white object and draw minimum enclosing circle
        contours_white, _ = cv2.findContours(mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_distance = np.inf
        min_distance_orange = np.inf
        closest_ball_center = None

        # Convert BGR to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply a blur to reduce noise
        image_blur = cv2.GaussianBlur(gray, (5, 5), 0)

        circles = cv2.HoughCircles(image_blur, cv2.HOUGH_GRADIENT, dp=1, minDist=30, param1=50, param2=20, minRadius=14,
                                  maxRadius=17)

        num_balls = 0
        if circles is not None:
            # Convert the coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype(int)

            # Filter for white colors and exclude dark colors
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower_white = np.array([0, 0, 100])  # Define lower threshold for white color
            upper_white = np.array([179, 255, 255])  # Define upper threshold for white color
            mask = cv2.inRange(hsv, lower_white, upper_white)

            # Apply the mask to the circles and keep only the white circles
            filtered_circles = []
            for (x, y, r) in circles:
                if mask[y, x] == 255:  # Check if the pixel is white
                    filtered_circles.append((x, y, r))

            # Draw the filtered circles
            num_balls = len(filtered_circles)
            for (x, y, r) in filtered_circles:
                cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
                center = (int(x), int(y))
                pixel_distance = calculate_distance(center, (cX, cY))
                pixel_distance = calculate_distance(center, (cX, cY))

                if pixel_distance < min_distance:
                    min_distance = pixel_distance
                    closest_ball_center = center

    pink_center_back = None
    if closest_ball_center is not None:
        # Draw a line between centroid of green object and center of the closest white ball
        cv2.line(frame, (cX, cY), closest_ball_center, (0, 0, 255), 2)

        # Convert pixel distance to cm and display it on the frame
        ball_distance = distance_cm = min_distance * conversion_factor
        cv2.putText(frame, f"Distance: {distance_cm:.2f} cm", (cX - 20, cY - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        back_distance = 100
        pink_center_back = calculate_new_coordinates(pink_center, angle_of_robot, back_distance)

        cv2.circle(frame, pink_center_back, 10, (255, 0, 255), -1)
        # Calculate and display angle between the two lines
        angle_to_ball = ball_angle = calculate_angle(pink_center_back, closest_ball_center)

        cv2.putText(frame, f"Angle to ball: {ball_angle:.2f}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

    ########################################### FINDING DISTANCE TO GOAL ##########################################

    # Draw the goals on the frame
    cv2.circle(frame, goal_left, radius=8, color=(0, 255, 255), thickness=-2)  # Yellow dot
    cv2.circle(frame, goal_right, radius=8, color=(0, 255, 255), thickness=-2)  # Yellow dot

    cv2.circle(frame, goal_point_left, radius=8, color=(0, 255, 255), thickness=-2)  # Yellow dot
    cv2.circle(frame, goal_point_right, radius=8, color=(0, 255, 255), thickness=-2)  # Yellow dot

    # Calculate distances to the goals
    distance_to_left_goal = calculate_distance(pink_center, goal_left) * conversion_factor
    distance_to_right_goal = calculate_distance(pink_center, goal_right) * conversion_factor

    distance_to_left_goal_point = calculate_distance(pink_center, goal_point_left) * conversion_factor
    distance_to_right_goal_point = calculate_distance(pink_center, goal_point_right) * conversion_factor

    goal_angle = None
    # Draw a line to the closest goal
    if distance_to_left_goal < distance_to_right_goal:
        angle_to_goal = goal_angle = calculate_angle(pink_center, goal_left)
        goal_distance = distance_to_left_goal
        draw_line_to_goals(frame, pink_center, goal_left, (0, 255, 255), thickness=2)

        goal_point_angle = calculate_angle(pink_center, goal_point_left)
        goal_point_distance = distance_to_left_goal_point
    else:
        angle_to_goal = goal_angle = calculate_angle(pink_center, goal_right)
        goal_distance = distance_to_right_goal
        draw_line_to_goals(frame, pink_center, goal_right, (0, 255, 255), thickness=2)

        goal_point_angle = calculate_angle(pink_center, goal_point_right)
        goal_point_distance = distance_to_right_goal_point

    cv2.putText(frame, f"Angle to goal: {goal_angle:.2f}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 0, 0), 2)
    cv2.putText(frame, f"distance to goal: {goal_distance:.2f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 0, 0), 2)

    cv2.imshow('All Contours', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        #flask_thread.join()
        break

cap.release()
cv2.destroyAllWindows()
