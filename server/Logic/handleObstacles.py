import cv2
import numpy as np

global top_left, top_right, bottom_right, bottom_left
global obstacle_points
obstacle_center = (int(0), int(0))


def get_obstacle_center():
    return obstacle_center


def avoid_obstacle(robot, obj, center):
    #find quadrants for robot and object
    robot_q = find_quadrant(robot, center)
    obj_q = find_quadrant(obj, center)

    # for debugging
    print("robottens kvadrant", robot_q)
    print("boldens kvadrant", obj_q)

    if abs(robot_q - obj_q) == 2:
        print('gå til nærmeste', )
        dest = get_closest_corner(robot)
        move_to = obstacle_points[dest - 1]

    elif robot_q == obj_q:
        print('gå hen til bold')
        move_to = obj
    else:
        print('gå til boldkvadrant  ', obj_q)
        dest = obj_q
        move_to = obstacle_points[dest - 1]

    # move to dest
    return move_to


def is_obstacle(line_start, line_end):
    global top_left, top_right, bottom_right, bottom_left

    # check intersection between each of the four rectangle lines and the robot route
    intersection1 = line_intersection(line_start, line_end, top_left, top_right)
    intersection2 = line_intersection(line_start, line_end, top_left, bottom_left)
    intersection3 = line_intersection(line_start, line_end, top_right, bottom_right)
    intersection4 = line_intersection(line_start, line_end, bottom_left, bottom_right)

    if intersection1 or intersection2 or intersection3 or intersection4:
        return True

    return False


def line_intersection(line1_start, line1_end, line2_start, line2_end):

    # calculate differences
    delta_p1p2 = (line1_end[0] - line1_start[0], line1_end[1] - line1_start[1])
    delta_p3p4 = (line2_end[0] - line2_start[0], line2_end[1] - line2_start[1])

    # calculate determinant
    det = delta_p1p2[0] * delta_p3p4[1] - delta_p1p2[1] * delta_p3p4[0]

    if abs(det) < 1e-6:
        return False

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

        # these are the size ranges for mac
        min_contour_area = 3450
        max_contour_area = 15000
        contour_area = cv2.contourArea(approx)

        if min_contour_area < contour_area < max_contour_area:
            middle_obstacle = approx

            # find minimum bounding rectangle that encloses the obstacle
            rect = cv2.minAreaRect(middle_obstacle)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            # define center of obstacle
            center_x = int(rect[0][0])
            center_y = int(rect[0][1])
            obstacle_center = (center_x, center_y)

            # scale to become bigger
            scale_factor = 3

            # rotate and scale the bounding rectangle
            rotation_matrix = cv2.getRotationMatrix2D((center_x, center_y), 45, scale_factor)
            rotated_box = cv2.transform(np.array([box]), rotation_matrix)[0]

            # assign the corner points to global values
            obstacle_points = (
            tuple(rotated_box[2]), tuple(rotated_box[1]), tuple(rotated_box[0]), tuple(rotated_box[3]))
            top_right = obstacle_points[0]
            top_left = obstacle_points[1]
            bottom_left = obstacle_points[2]
            bottom_right = obstacle_points[3]

            # draw rectangle
    if rotated_box is not None:
        cv2.drawContours(image, [rotated_box], 0, (0, 0, 255), 2)

    # draw dot at center
    cv2.circle(image, (center_x, center_y), 3, (0, 255, 0), -1)


# Returns which coordinate an object is from the center.
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
        # calculate distance between the robot and each corner
        distance = ((corner[0] - robot_coordinate[0]) ** 2 + (corner[1] - robot_coordinate[1]) ** 2) ** 0.5
        distances.append(distance)

    # find the index of the closest corner
    closest_corner = distances.index(min(distances))

    # return the closest corner
    return closest_corner


def detect_obstacle(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # define color range for obstacle
    lower_red1 = np.array([0, 70, 50])
    upper_red1 = np.array([20, 255, 255])
    lower_red2 = np.array([160, 70, 50])
    upper_red2 = np.array([180, 255, 255])

    # create mask for red color
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    # enhance the mask
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)

    # find contours of the red clusters
    contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # draw contours on copied image
    image_with_contours = image.copy()
    obstacle_contours = []

    for contour in contours:
        # approximate contour as a polygon
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # if polygon has 12 sides it is the middle obstacle
        if len(approx) == 12:
            obstacle_contours.append(approx)  # Append the approximated contour

    # draws rectangle and obstacle center
    draw_rect_and_center(image_with_contours, obstacle_contours)

    return obstacle_contours, image_with_contours


