import cv2
import numpy as np

global top_left, top_right, bottom_right, bottom_left
global obstacle_points
global obstacle_center


def get_obstacle_center():
    return obstacle_center

# skal måske ændres ift hvordan koordinaterne er for framen.
def avoid_obstacle(robot, ball, center):
    robot_q = find_quadrant(robot, center)
    ball_q = find_quadrant(ball, center)

    print("robottens kvadrant", robot_q)
    print("boldens kvadrant", ball_q)

    if abs(robot_q - ball_q) == 2:
        print('gå til nærmeste', )

        dest = get_closest_corner(robot)
    else:
        print('gå til boldkvadrant  ', ball_q)

        dest = ball_q

    # move to dest
    move_to = obstacle_points[dest - 1]
    return move_to


def is_obstacle(line_start, line_end):
    global top_left, top_right, bottom_right, bottom_left

    # Check if the line intersects with any of the edges of the rectangle
    intersection1 = line_intersection(line_start, line_end, top_left, top_right)
    intersection2 = line_intersection(line_start, line_end, top_left, bottom_left)
    intersection3 = line_intersection(line_start, line_end, top_right, bottom_right)
    intersection4 = line_intersection(line_start, line_end, bottom_left, bottom_right)

    if intersection1 or intersection2 or intersection3 or intersection4:
        return True

    return False


def line_intersection(line1_start, line1_end, line2_start, line2_end):

    # Calculate the differences
    delta_p1p2 = (line1_end[0] - line1_start[0], line1_end[1] - line1_start[1])
    delta_p3p4 = (line2_end[0] - line2_start[0], line2_end[1] - line2_start[1])

    # Calculate the determinant
    det = delta_p1p2[0] * delta_p3p4[1] - delta_p1p2[1] * delta_p3p4[0]

    # Check if the lines are parallel or coincident
    if abs(det) < 1e-6:
        return False

    # Calculate the parameters for the line equations
    delta_p3p1 = (line1_start[0] - line2_start[0], line1_start[1] - line2_start[1])
    t = (delta_p3p4[0] * delta_p3p1[1] - delta_p3p4[1] * delta_p3p1[0]) / det
    u = (-delta_p1p2[0] * delta_p3p1[1] + delta_p1p2[1] * delta_p3p1[0]) / det

    # Check if the intersection point is within the line segments
    if 0 <= t <= 1 and 0 <= u <= 1:
        return True

    return False


def draw_rect_and_center(image, contours):
    global top_left, top_right, bottom_right, bottom_left, obstacle_points, obstacle_center
    rotated_box = None
    center_x = None
    center_y = None

    for contour in contours:
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Check if the contour has a certain size range
        min_contour_area = 4000  # Adjust this value as needed
        max_contour_area = 15000  # Adjust this value as needed
        contour_area = cv2.contourArea(approx)

        if min_contour_area < contour_area < max_contour_area:
            middle_obstacle = approx

            # Find the minimum bounding rectangle that encloses the contour
            rect = cv2.minAreaRect(middle_obstacle)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            # Calculate the center of the bounding rectangle
            center_x = int(rect[0][0])
            center_y = int(rect[0][1])

            obstacle_center = (center_x, center_y)

            # Calculate the rotation angle and scale factor
            angle = 45
            scale_factor = 3

            # Rotate and scale the bounding rectangle
            rotation_matrix = cv2.getRotationMatrix2D((center_x, center_y), angle, scale_factor)
            rotated_box = cv2.transform(np.array([box]), rotation_matrix)[0]

            # Get the corner points of the rotated and scaled rectangle
            obstacle_points = (
            tuple(rotated_box[2]), tuple(rotated_box[1]), tuple(rotated_box[0]), tuple(rotated_box[3]))
            top_right = obstacle_points[0]
            top_left = obstacle_points[1]
            bottom_left = obstacle_points[2]
            bottom_right = obstacle_points[3]

            # Draw the rotated and scaled bounding rectangle
    if rotated_box is not None:
        cv2.drawContours(image, [rotated_box], 0, (0, 0, 255), 2)

    # Draw a circle to represent the center of the bounding rectangle
    cv2.circle(image, (center_x, center_y), 3, (0, 255, 0), -1)


def lineIntersection1(x1, y1, x2, y2):
    #p1 og p2 er robot. p3 og p4 er dest_punkt
    for point in obstacle_points:
        x3 = obstacle_points[point][0]
        y3 = obstacle_points[(point+1)%4][0]
        x4 = obstacle_points[point][1]
        y4 = obstacle_points[(point+1)%4][1]

        denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

        if denominator != 0:
            # Calculate the intersection point coordinates
            intersection_x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denominator
            intersection_y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denominator

            return int(intersection_x), int(intersection_y)
        else:
            # Lines are parallel or coincident, no intersection exists
            return None



def find_quadrant(obj_coordinate, center_coordinate):
    if obj_coordinate[0] > center_coordinate[0] and obj_coordinate[1] > center_coordinate[1]:
        return 1
    elif obj_coordinate[0] < center_coordinate[0] and obj_coordinate[1] > center_coordinate[1]:
        return 2
    elif obj_coordinate[0] < center_coordinate[0] and obj_coordinate[1] < center_coordinate[1]:
        return 3
    elif obj_coordinate[0] > center_coordinate[0] and obj_coordinate[1] < center_coordinate[1]:
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
    return closest_corner_index


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
    upper_red1 = np.array([20, 255, 255])
    lower_red2 = np.array([160, 70, 50])
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
            obstacle_contours.append(approx)  # Append the approximated contour

    # Draw contours on the image
    draw_rect_and_center(image_with_contours, obstacle_contours)

    return obstacle_contours, image_with_contours


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


