import cv2
import numpy as np

cap = cv2.VideoCapture(0)

lower_green = np.array([30, 30, 30], dtype=np.uint8)
upper_green = np.array([80, 255, 255], dtype=np.uint8)

while True:
    ret, frame = cap.read()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_green, upper_green)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=1)

    blur = cv2.blur(mask,(14,14))

    contours, hierarchy= cv2.findContours(blur, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    
    # Sort contours by area (Assume that the largest contour is the robot)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    if contours:
        # Get the moments of the largest contour
        M = cv2.moments(contours[0])
        # Calculate the centroid of the contour from the moments
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        
        # Draw a circle at the centroid
        cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)
        cv2.putText(frame, f"centroid {cX}, {cY}", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    cv2.imshow('All Contours', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
