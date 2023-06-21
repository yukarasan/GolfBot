import cv2
import numpy as np

cap = cv2.VideoCapture(0)

lower_white = np.array([0, 0, 200])
upper_white = np.array([180, 30, 255])

while True:
    ret, frame = cap.read()
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    gray = cv2.cvtColor(hsv, cv2.COLOR_BGR2GRAY)
    
    mask = cv2.inRange(hsv, lower_white, upper_white)

    kernel = np.ones((5, 5), np.uint8)

    # mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=1)
    cv2.imshow('Sidste mask', mask)

    contours, hierarchy= cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    
    for cnt in contours:
        if cnt.shape[0] > 10:
            (x,y),radius = cv2.minEnclosingCircle(cnt)
            center = (int(x),int(y))
            radius = int(radius)
            
            if radius > 8 and radius < 20:
                cv2.circle(frame,center,radius,(0,255,0),2)
                prevCircle = center + (radius,)
    
    
    cv2.imshow('All Contours', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()