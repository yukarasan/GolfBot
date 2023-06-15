import cv2
import numpy as np

global top_left, top_right, bottom_right, bottom_left
global obstacle_points


# skal måske ændres ift hvordan koordinaterne er for framen.
def avoid_obstacle(robot, ball, center):
    robot_q = find_quadrant(robot, center)
    ball_q = find_quadrant(ball, center)

    if abs(robot_q - ball_q) == 2:
        print('gå til nærmeste')
        dest = get_closest_corner(robot)
        return
    else:
        print('gå til boldkvadrant')
        dest = ball_q

    # move to dest
    move_to = obstacle_points[dest - 1]
    return move_to


def is_obstacle(line_start, line_end):
    global top_left, top_right, bottom_right, bottom_left

    # Check if the line intersects with any of the edges of the rectangle
    intersection1 = cv2.lineIntersection(line_start, line_end, top_left, top_right)
    intersection2 = cv2.lineIntersection(line_start, line_end, top_left, bottom_left)
    intersection3 = cv2.lineIntersection(line_start, line_end, top_right, bottom_right)
    intersection4 = cv2.lineIntersection(line_start, line_end, bottom_left, bottom_right)

    if intersection1[0] or intersection2[0] or intersection3[0] or intersection4[0]:
        return True

    return False


def draw_rect_and_center(image, contour):
    global top_left, top_right, bottom_right, bottom_left
    global obstacle_points

    # Find the minimum bounding rectangle that encloses the contour
    rect = cv2.minAreaRect(contour)
    box = cv2.boxPoints(rect)
    box = np.int0(box)

    # Calculate the center of the bounding rectangle
    center_x = int(rect[0][0])
    center_y = int(rect[0][1])

    # Calculate the rotation angle and scale factor
    angle = 45
    scale_factor = 1.8

    # Rotate and scale the bounding rectangle
    rotation_matrix = cv2.getRotationMatrix2D((center_x, center_y), angle, scale_factor)
    rotated_box = cv2.transform(np.array([box]), rotation_matrix)[0]

    # Get the corner points of the rotated and scaled rectangle
    obstacle_points = (tuple(rotated_box[2]), tuple(rotated_box[1]), tuple(rotated_box[0]), tuple(rotated_box[3]))
    top_right = obstacle_points[0]
    top_left = obstacle_points[1]
    bottom_left = obstacle_points[2]
    bottom_right = obstacle_points[3]

    # Draw the rotated and scaled bounding rectangle
    cv2.drawContours(image, [rotated_box], 0, (0, 0, 255), 2)

    # Draw a circle to represent the center of the bounding rectangle
    cv2.circle(image, (center_x, center_y), 3, (0, 255, 0), -1)


def find_quadrant(obj_coordinate, center_coordinate):
    x_new = obj_coordinate[0] - center_coordinate[0]
    y_new = obj_coordinate[1] - center_coordinate[1]

    if x_new > 0 and y_new > 0:
        return 1
    elif x_new < 0 < y_new:
        return 2
    elif x_new < 0 and y_new < 0:
        return 3
    elif x_new > 0 > y_new:
        return 4


def get_closest_corner(robot_coordinate):
    distances = []
    corners = [top_left, top_right, bottom_right, bottom_left]

    for corner in corners:
        # Calculate the Euclidean distance between the robot coordinate and each corner
        distance = ((corner[0] - robot_coordinate[0]) ** 2 + (corner[1] - robot_coordinate[1]) ** 2) ** 0.5
        distances.append(distance)

    # Find the index of the minimum distance
    closest_corner_index = distances.index(min(distances))

    # Return the closest corner
    return corners[closest_corner_index]


def is_robot_close_to_obstacle(robot_contour, square_contour):
    # Check for intersection between the robot contour and the square contour
    intersection = cv2.bitwise_and(robot_contour, square_contour)

    # Check if there are any non-zero pixels in the intersection
    is_close = cv2.countNonZero(intersection) > 0

    # Return True if the robot is touching the square contour
    return is_close


def detect_obstacle(image):
    # Convert the image to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the lower and upper red color ranges
    lower_red1 = np.array([0, 70, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 70, 50])
    upper_red2 = np.array([180, 255, 255])

    # Create a mask for the red color range
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    # Apply morphological operations to enhance the mask
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)

    # Find contours of the red regions
    contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw contours on the image
    image_with_contours = image.copy()
    obstacle_contours = []

    for contour in contours:
        # Approximate the contour as a polygon
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Check if the contour is approximately a plus-sign shape
        if len(approx) == 12:
            obstacle_contours.append(contour)
            draw_rect_and_center(image_with_contours, contour)

    #cv2.drawContours(image_with_contours, obstacle_contours, -1, (0, 255, 0), 2)

    if len(obstacle_contours) > 0:
        return True, image_with_contours

    return False, image_with_contours


def make_obstacle_contours(image):
    # Load the sample image for testing
    sample_image = image
    # Resize the image to fit the screen
    scale_percent = 30  # Adjust the scale factor as needed
    width = int(sample_image.shape[1] * scale_percent / 100)
    height = int(sample_image.shape[0] * scale_percent / 100)
    resized_image = cv2.resize(sample_image, (width, height))

    # Call the detect_obstacle function with the resized image
    image_with_obstacles = detect_obstacle(resized_image)
    return image_with_obstacles

    # Show the resized image with obstacles
    #cv2.imshow("Obstacles", image_with_obstacles)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()


