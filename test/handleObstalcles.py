import cv2
import numpy as np

# Constants
DISTANCE_THRESHOLD = 2  # Distance threshold for stopping in cm


def draw_rect_and_center(image, contour):
    # Calculate the bounding rectangle around the contour
    x, y, width, height = cv2.boundingRect(contour)

    # Calculate the center of the bounding rectangle
    center_x = int(x + width / 2)
    center_y = int(y + height / 2)

    # Draw the bounding rectangle
    cv2.rectangle(image, (x - 20, y - 20), (x + width + 20, y + height + 20), (0, 0, 255), 2)

    # Draw a circle to represent the center of the bounding rectangle
    cv2.circle(image, (center_x, center_y), 3, (0, 255, 0), -1)


def find_quadrant(ball_coordinate, center_coordinate):
    x_new = ball_coordinate[0] - center_coordinate[0]
    y_new = ball_coordinate[1] - center_coordinate[1]

    if x_new > 0 and y_new > 0:
        return 1
    elif x_new < 0 < y_new:
        return 2
    elif x_new < 0 and y_new < 0:
        return 3
    elif x_new > 0 > y_new:
        return 4


def is_robot_close_to_obstacle(robot_contour, square_contour):
    # Check for intersection between the robot contour and the square contour
    intersection = cv2.bitwise_and(robot_contour, square_contour)

    # Check if there are any non-zero pixels in the intersection
    is_close = cv2.countNonZero(intersection) > 0

    # Return True if the robot is touching the square contour
    return is_close


# lav funktion til at gå hen til bold koordinat rundt om forhindring.


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

    cv2.drawContours(image_with_contours, obstacle_contours, -1, (0, 255, 0), 2)

    if len(obstacle_contours) > 0:
        return True, image_with_contours

    return False, image_with_contours


# Load the sample image for testing
sample_image = cv2.imread('C:/Users/mathi/PycharmProjects/GolfBot/images/testimage1.jpg')

# Resize the image to fit the screen
scale_percent = 50  # Adjust the scale factor as needed
width = int(sample_image.shape[1] * scale_percent / 100)
height = int(sample_image.shape[0] * scale_percent / 100)
resized_image = cv2.resize(sample_image, (width, height))

# Call the detect_obstacle function with the resized image
obstacle_detected, image_with_obstacles = detect_obstacle(resized_image)

# Show the resized image with obstacles
cv2.imshow("Obstacles", image_with_obstacles)
cv2.waitKey(0)
cv2.destroyAllWindows()
