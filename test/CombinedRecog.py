import cv2 as cv
import numpy as np

videoCapture = cv.VideoCapture(0)
prevCircle = None

lower_white = np.array([0, 0, 200])
upper_white = np.array([180, 30, 255])

red_lower = np.array([0, 100, 100])
red_upper = np.array([10, 255, 255])
    
lower_green = np.array([30, 30, 30], dtype=np.uint8)
upper_green = np.array([80, 255, 255], dtype=np.uint8)

while True:
    ret, frame = videoCapture.read()
    if not ret: break
   
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower_white, upper_white)

    kernel = np.ones((5,5),np.uint8)
    mask = cv.erode(mask,kernel,iterations = 1)
    mask = cv.dilate(mask,kernel,iterations = 1)
    
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        if cnt.shape[0] > 5:
            (x,y),radius = cv.minEnclosingCircle(cnt)
            center = (int(x),int(y))
            radius = int(radius)
            
            if radius > 8 and radius < 20:
                cv.circle(frame,center,radius,(0,255,0),2)
                prevCircle = center + (radius,)
                    
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    redmask = cv.inRange(hsv, red_lower, red_upper)
    gray = cv.cvtColor(hsv, cv.COLOR_BGR2GRAY)
    kernel = np.ones((5, 5), np.uint8)
    redmask = cv.erode(redmask, kernel, iterations=1)
    redmask = cv.dilate(redmask, kernel, iterations=1)
    redblur = cv.blur(redmask,(14,14))
    redcontours, hierarchy= cv.findContours(redblur, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
    cv.drawContours(frame, redcontours, -1, (0,255,0),3)
    
    cv.imshow('red', redblur)
    
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    rmask = cv.inRange(hsv, lower_green, upper_green)
    gray = cv.cvtColor(hsv, cv.COLOR_BGR2GRAY)
    kernel = np.ones((5, 5), np.uint8)
    rmask = cv.erode(rmask, kernel, iterations=1)
    rmask = cv.dilate(rmask, kernel, iterations=1)
    rblur = cv.blur(rmask,(14,14))
    edges = cv.Canny(rblur,30,200)
    contours, hierarchy= cv.findContours(edges, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
    cv.drawContours(frame, contours, -1, (0,255,0),3)
    
    
    cv.imshow('robot', edges)
    cv.imshow('circles', frame)
    
    if cv.waitKey(1) & 0xFF == ord('q'): break

videoCapture.release()
cv.destroyAllWindows()