import cv2 as cv
import numpy as np

videoCapture = cv.VideoCapture(0)
prevCircle = None

while True:
    ret, frame = videoCapture.read()
    if not ret:
        break

    # Convert BGR to HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # Define range of white color in HSV
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 30, 255])

    # Threshold the HSV image to get only white colors
    mask_white = cv.inRange(hsv, lower_white, upper_white)

    # Define range of orange color in HSV
    lower_orange = np.array([20, 100, 100])
    upper_orange = np.array([30, 255, 255])

    # Threshold the HSV image to get only orange colors
    mask_orange = cv.inRange(hsv, lower_orange, upper_orange)

    # Combine the masks
    mask = cv.bitwise_or(mask_white, mask_orange)

    # Apply some morphological operations to the mask to remove noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv.erode(mask, kernel, iterations=1)
    mask = cv.dilate(mask, kernel, iterations=1)

    cv.imshow('last mask', mask)

    # Find contours in the mask and loop over them
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        # Fit a circle to the contour if it has enough points
        if cnt.shape[0] > 5:
            (x, y), radius = cv.minEnclosingCircle(cnt)
            center = (int(x), int(y))
            radius = int(radius)

            # Draw the circle if it's big enough and track it
            if radius > 8 and radius < 20:
                cv.circle(frame, center, radius, (0, 255, 0), 2)
                prevCircle = center + (radius,)

    cv.imshow('circles', frame)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

videoCapture.release()
cv.destroyAllWindows()