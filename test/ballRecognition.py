import cv2 as cv
import numpy as np

videoCapture = cv.VideoCapture(0)
prevCircle = None

while True:
    ret, frame = videoCapture.read()
    if not ret:
        break

    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    image_blur = cv.GaussianBlur(gray, (5, 5), 0)

    circles = cv.HoughCircles(image_blur, cv.HOUGH_GRADIENT, dp=1, minDist=30, param1=50, param2=20, minRadius=15,
                               maxRadius=18)

    if circles is not None:
        circles = np.round(circles[0, :]).astype(int)

        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 100])  
        upper_white = np.array([179, 255, 255])
        mask = cv.inRange(hsv, lower_white, upper_white)

        filtered_circles = []
        for (x, y, r) in circles:
            if mask[y, x] == 255:  
                filtered_circles.append((x, y, r))

        for (x, y, r) in filtered_circles:
            cv.circle(frame, (x, y), r, (0, 255, 0), 2)

    cv.imshow('circles', frame)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

videoCapture.release()
cv.destroyAllWindows()