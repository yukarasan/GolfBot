import cv2
import numpy as np

cap = cv2.VideoCapture(0)

red_lower = np.array([0, 100, 100])
red_upper = np.array([10, 255, 255])

lower_white = np.array([0, 0, 200])
upper_white = np.array([180, 30, 255])


while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, red_lower, red_upper)
    
    gray = cv2.cvtColor(hsv, cv2.COLOR_BGR2GRAY)
    
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=1)
    
    #cv2.imshow('gray', mask)
    
    blur = cv2.blur(mask,(14,14))
    
    edges = cv2.Canny(blur,30,200)
    contours, hierarchy= cv2.findContours(blur, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(frame, contours, -1, (0,255,0),3)
    cv2.imshow('All Contours', frame)

    # Exit the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture device and destroy all windows
cap.release()
cv2.destroyAllWindows()