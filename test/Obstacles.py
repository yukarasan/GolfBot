import cv2
import numpy as np

def calculate_distance(pt1, pt2):
    # Calculate Euclidean distance between two points
    return np.sqrt((pt1[0] - pt2[0]) ** 2 + (pt1[1] - pt2[1]) ** 2)

# Range for red
red_lower = np.array([0, 100, 100])
red_upper = np.array([10, 255, 255])

# Assuming you have already obtained the contours of the red wall

# Calculate the Euclidean distance between the ball and the red wall
min_distance = float('inf')
closest_point = None

# Open the webcam
cap = cv2.VideoCapture(0)

kernel = np.ones((5, 5), np.uint8)

while True:
    ret, frame = cap.read()

    if not ret:
        break

    # Convert BGR to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply a blur to reduce noise
    image_blur = cv2.GaussianBlur(gray, (5, 5), 0)

    circles = cv2.HoughCircles(image_blur, cv2.HOUGH_GRADIENT, dp=1, minDist=30, param1=50, param2=20, minRadius=14,
                               maxRadius=17)
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

        min_distance = 200
        # Draw the filtered circles
        for (x, y, r) in filtered_circles:
            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
            center = (int(x), int(y))
            pixel_distance = calculate_distance(center, (4, 4))

            if True:
                min_distance = pixel_distance
                closest_ball_center = center
                print("closest ball sat")

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_red = cv2.inRange(hsv, red_lower, red_upper)
    mask_red = cv2.erode(mask_red, kernel, iterations=1)
    mask_red = cv2.dilate(mask_red, kernel, iterations=1)

    # Blur mask for red object
    blur_red = cv2.blur(mask_red, (14, 14))

    # Find contours for red object
    contours_red, _ = cv2.findContours(blur_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    cv2.drawContours(frame, contours_red, -1, (0, 0, 255), 2)

    for contour in contours_red:
        for point in contour:
            distance = np.sqrt((point[0][0] - closest_ball_center[0]) ** 2 + (point[0][1] - closest_ball_center[1]) ** 2)
            print("distance", distance)

            if distance < min_distance:
                print("tæt på")
                min_distance = distance
                closest_point = point[0]

    # Check if the ball is within a distance of 10 from the red wall
    if closest_point is not None and min_distance <= 2000:
        # Calculate the vector pointing in the opposite direction
        opposite_vector = np.array([closest_point[0] - closest_ball_center[0], closest_point[1] - closest_ball_center[1]])
        # Normalize the vector
        opposite_vector = opposite_vector.astype(float)  # Convert to float
        # Normalize the vector
        opposite_vector /= np.linalg.norm(opposite_vector)

        # Multiply by -1 to make it point in the opposite direction
        opposite_vector *= -1

        # Draw a circle around the closest point
        cv2.circle(frame, (closest_point[0], closest_point[1]), 10, (0, 255, 0), 2)
    else:
        opposite_vector = None

    # Draw the vector on the frame (if applicable)
    if opposite_vector is not None:
        # Calculate the end point of the vector
        end_point = (int(closest_ball_center[0] + 220 * opposite_vector[0]), int(closest_ball_center[1] + 220 * opposite_vector[1]))

        # Draw a line from the ball to the end point of the vector
        cv2.arrowedLine(frame, closest_ball_center, end_point, (0, 0, 255), 2)

    # Display the frame
    cv2.imshow("Frame", frame)

    # Check for the 'q' key to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close any open windows
cap.release()
cv2.destroyAllWindows()
