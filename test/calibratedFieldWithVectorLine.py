import cv2
import numpy as np
import math

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

kernel = np.ones((5, 5), np.uint8)

def calculate_angle_between_lines(p1, p2, p3, p4):
    # Calculate slopes
    m1 = (p2[1] - p1[1]) / (p2[0] - p1[0])
    m2 = (p4[1] - p3[1]) / (p4[0] - p3[0])

    # Calculate angle
    tan_theta = abs((m2 - m1) / (1 + m1 * m2))
    angle = math.degrees(math.atan(tan_theta))

    return angle

def calculate_distance(pt1, pt2):
    # Calculate Euclidean distance between two points
    return np.sqrt((pt1[0] - pt2[0]) ** 2 + (pt1[1] - pt2[1]) ** 2)

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

# Initialize conversion factor
conversion_factor = None

while True:
    ret, frame = cap.read()

    if not ret:
        break

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

    # Threshold the image to get only blue and green regions
    blue_mask = cv2.inRange(hsv, lower_pink, upper_pink)
    green_mask = cv2.inRange(hsv, lower_green, upper_green)

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
            green_center = (
            int(green_moment["m10"] / green_moment["m00"]), int(green_moment["m01"] / green_moment["m00"]))

            # Calculate the angle between the centers of blue and green rectangles
            angle = calculate_angle(blue_center, green_center)

            # Draw a line connecting the centers of blue and green rectangles
            draw_line(frame, blue_center, green_center, (0, 255, 0), thickness=2)

            # Display the angle on the frame
            cv2.putText(frame, "Angle: {:.2f}".format(angle), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)


    # Apply morphological operations
    mask_green = cv2.erode(mask_green, kernel, iterations=1)
    mask_green = cv2.dilate(mask_green, kernel, iterations=1)
    mask_white = cv2.erode(mask_white, kernel, iterations=1)
    mask_white = cv2.dilate(mask_white, kernel, iterations=1)
    mask_red = cv2.erode(mask_red, kernel, iterations=1)
    mask_red = cv2.dilate(mask_red, kernel, iterations=1)

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
        box = np.int0(box)

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

    if contours_green:
        M = cv2.moments(contours_green[0])
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)
        cv2.putText(frame, f"centroid {cX}, {cY}", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255),
                    2)

        # Find white object and draw minimum enclosing circle
        contours_white, _ = cv2.findContours(mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_distance = np.inf
        closest_ball_center = None

        for cnt in contours_white:
            if cnt.shape[0] > 5:
                (x, y), radius = cv2.minEnclosingCircle(cnt)
                center = (int(x), int(y))
                radius = int(radius)
                if radius > 8 and radius < 20:
                    cv2.circle(frame, center, radius, (0, 255, 0), 2)
                    cv2.putText(frame, f"ball {center[0]}, {center[1]}", (center[0] - 20, center[1] - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # Calculate distance between centroid of green object and center of white ball
                    pixel_distance = calculate_distance(center, (cX, cY))
                    if pixel_distance < min_distance:
                        min_distance = pixel_distance
                        closest_ball_center = center

        if closest_ball_center is not None:
            # Draw a line between centroid of green object and center of the closest white ball
            cv2.line(frame, (cX, cY), closest_ball_center, (0, 0, 255), 2)

            # Convert pixel distance to cm and display it on the frame
            distance_cm = min_distance * conversion_factor
            cv2.putText(frame, f"Distance: {distance_cm:.2f} cm", (cX - 20, cY - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 0, 255), 2)

            # Calculate and display angle between the two lines
            angle_between_lines = calculate_angle_between_lines(blue_center, green_center, (cX, cY),
                                                                closest_ball_center)
            cv2.putText(frame, f"Angle between lines: {angle_between_lines:.2f}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (255, 0, 0), 2)

    cv2.imshow('All Contours', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()