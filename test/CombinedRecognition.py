import cv2 as cv
import numpy as np

# Define video capture device
videoCapture = cv.VideoCapture(0)

# Define color ranges
white_lower = np.array([0, 0, 200])
white_upper = np.array([180, 30, 255])

red_lower = np.array([0, 100, 100])
red_upper = np.array([10, 255, 255])

green_lower = np.array([30, 30, 30], dtype=np.uint8)
green_upper = np.array([80, 255, 255], dtype=np.uint8)

# Initialize previous circle and rectangle
prevCircle = None
prevRect = None

while True:
    # Read frame from video capture
    ret, frame = videoCapture.read()
    if not ret: break
    
    # Convert frame to grayscale
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # Detect white circles
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    white_mask = cv.inRange(hsv, white_lower, white_upper)
    white_mask = cv.erode(white_mask, np.ones((5,5), np.uint8), iterations=1)
    white_mask = cv.dilate(white_mask, np.ones((5,5), np.uint8), iterations=1)
    white_contours, _ = cv.findContours(white_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    for cnt in white_contours:
        if cnt.shape[0] > 5:
            (x,y),radius = cv.minEnclosingCircle(cnt)
            center = (int(x),int(y))
            radius = int(radius)
            
            if radius > 8 and radius < 20:
                cv.circle(frame,center,radius,(0,255,0),2)
                prevCircle = center + (radius,)

    # Detect red rectangles
    red_mask = cv.inRange(hsv, red_lower, red_upper)
    red_mask = cv.erode(red_mask, np.ones((5,5), np.uint8), iterations=1)
    red_mask = cv.dilate(red_mask, np.ones((5,5), np.uint8), iterations=1)
    red_contours, _ = cv.findContours(red_mask, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
    for cnt in red_contours:
        area = cv.contourArea(cnt)
        if area > 500:
            approx = cv.approxPolyDP(cnt, 0.02*cv.arcLength(cnt, True), True)
            if len(approx) == 4:
                x, y, w, h = cv.boundingRect(approx)
                if prevRect is not None:
                    if (x < prevRect[0] + 20 and x > prevRect[0] - 20) and (y < prevRect[1] + 20 and y > prevRect[1] - 20):
                        cv.rectangle(frame, (x, y), (x+w, y+h), (0,0,255), 2)
                        prevRect = (x,y,w,h)
                        break
                else:
                    cv.rectangle(frame, (x, y), (x+w, y+h), (0,0,255), 2)
                    prevRect = (x,y,w,h)
                    break

    # Detect green rectangles
    green_mask = cv.inRange(hsv, green_lower, green_upper)
    green_mask = cv.erode(green_mask, np.ones((5,5), np.uint8), iterations=1)
    green_mask = cv.dilate(green_mask, np.ones((5,5), np.uint8), iterations=1)
    green_contours, _ = cv.findContours(green_mask, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
    for cnt in green_contours:
        area = cv.contourArea(cnt)
        if area > 500:
            approx = cv.approxPolyDP(cnt, 0.02*cv.arcLength(cnt, True), True)
            if len(approx) == 4:
                x, y, w, h = cv.boundingRect(approx)
                cv.rectangle(frame, (x, y), (x+w, y+h), (0,255,0), 2)
                
                
    cv.imshow('All Contours', frame)
                
                
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture device and destroy all windows
cap.release()
cv.destroyAllWindows()


