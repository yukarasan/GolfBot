import cv2 as cv
import numpy as np

videoCapture = cv.VideoCapture(0)
prevCircle = None

while True:
    ret, frame = videoCapture.read()
    if not ret:
        break

    # Convert BGR to grayscale
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # Apply a blur to reduce noise
    image_blur = cv.GaussianBlur(gray, (5, 5), 0)

    circles = cv.HoughCircles(image_blur, cv.HOUGH_GRADIENT, dp=1, minDist=30, param1=50, param2=20, minRadius=15,
                               maxRadius=18)

    if circles is not None:
        # Convert the coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype(int)

        # Filter for white colors and exclude dark colors
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 100])  # Define lower threshold for white color
        upper_white = np.array([179, 255, 255])  # Define upper threshold for white color
        mask = cv.inRange(hsv, lower_white, upper_white)

        # Apply the mask to the circles and keep only the white circles
        filtered_circles = []
        for (x, y, r) in circles:
            if mask[y, x] == 255:  # Check if the pixel is white
                filtered_circles.append((x, y, r))

        # Draw the filtered circles
        for (x, y, r) in filtered_circles:
            cv.circle(frame, (x, y), r, (0, 255, 0), 2)

    cv.imshow('circles', frame)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

videoCapture.release()
cv.destroyAllWindows()
