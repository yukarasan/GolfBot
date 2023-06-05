import cv2
import numpy as np
import math
from server import setUpSocketAndReturnIt, waitForClientAndReturnClientSocket, sendMessageINS

def calculate_angle(center1, center2):
    x1, y1 = center1
    x2, y2 = center2
    angle = math.degrees(math.atan2(y2-y1, x2-x1))
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

cap = cv2.VideoCapture(0)

server_socket = setUpSocketAndReturnIt()


while True:

    ret, frame = cap.read()
    frame = cv2.flip(frame, 1)
    if not ret:
        break

    # Convert frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define lower and upper bounds for blue color
    lower_pink = np.array([150, 50, 50])
    upper_pink = np.array([180, 255, 255])

    # Define lower and upper bounds for green color
    lower_green = np.array([40, 50, 50])
    upper_green = np.array([80, 255, 255])

    # Threshold the image to get only blue and green regions
    blue_mask = cv2.inRange(hsv_frame, lower_pink, upper_pink)
    green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)

    # Find contours for blue and green regions
    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Find the largest blue and green contours
    blue_contour = max(blue_contours, key=cv2.contourArea) if blue_contours else None
    green_contour = max(green_contours, key=cv2.contourArea) if green_contours else None

    if blue_contour is not None and green_contour is not None:
        # Get the center points of blue and green rectangles
        blue_moment = cv2.moments(blue_contour)
        green_moment = cv2.moments(green_contour)

        if blue_moment["m00"] != 0 and green_moment["m00"] != 0:
            blue_center = (int(blue_moment["m10"] / blue_moment["m00"]), int(blue_moment["m01"] / blue_moment["m00"]))
            green_center = (int(green_moment["m10"] / green_moment["m00"]), int(green_moment["m01"] / green_moment["m00"]))

            # Calculate the angle between the centers of blue and green rectangles
            angle = calculate_angle(blue_center, green_center)

            # Draw a line connecting the centers of blue and green rectangles
            draw_line(frame, blue_center, green_center, (0, 255, 0), thickness=2)

            # Display the angle on the frame
            cv2.putText(frame, "Angle: {:.2f}".format(angle), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # Display the frames
    cv2.imshow("Rectangles and Angle Detection", frame)

    client_socket = waitForClientAndReturnClientSocket(server_socket)

    sendMessageINS(angle, client_socket)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
