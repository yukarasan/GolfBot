import cv2
import numpy as np

cap = cv2.VideoCapture(0)

lower_green = np.array([30, 30, 30], dtype=np.uint8)
upper_green = np.array([80, 255, 255], dtype=np.uint8)
lower_white = np.array([0, 0, 200])
upper_white = np.array([180, 30, 255])
red_lower = np.array([0, 100, 100])
red_upper = np.array([10, 255, 255])

kernel = np.ones((5, 5), np.uint8)

real_width_cm = 10  
width_in_pixels = 100 

conversion_factor = real_width_cm / width_in_pixels

def calculate_distance(pt1, pt2):
    return np.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)

while True:
    ret, frame = cap.read()
    frame = cv2.flip(frame, 1)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_white = cv2.inRange(hsv, lower_white, upper_white)
    mask_red = cv2.inRange(hsv, red_lower, red_upper)

    mask_green = cv2.erode(mask_green, kernel, iterations=1)
    mask_green = cv2.dilate(mask_green, kernel, iterations=1)
    mask_white = cv2.erode(mask_white, kernel, iterations=1)
    mask_white = cv2.dilate(mask_white, kernel, iterations=1)
    mask_red = cv2.erode(mask_red, kernel, iterations=1)
    mask_red = cv2.dilate(mask_red, kernel, iterations=1)
    
    blur_red = cv2.blur(mask_red,(14,14))

    contours_red, _ = cv2.findContours(blur_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(frame, contours_red, -1, (0,255,0),3)

    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    contours_green = sorted(contours_green, key=cv2.contourArea, reverse=True)

    if contours_green:
        M = cv2.moments(contours_green[0])
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)
        cv2.putText(frame, f"centroid {cX}, {cY}", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

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
                    cv2.putText(frame, f"ball {center[0]}, {center[1]}", (center[0] - 20, center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    pixel_distance = calculate_distance(center, (cX, cY))
                    if pixel_distance < min_distance:
                        min_distance = pixel_distance
                        closest_ball_center = center

        if closest_ball_center is not None:
            cv2.line(frame, (cX, cY), closest_ball_center, (0, 0, 255), 2)

            distance_cm = min_distance * conversion_factor
            cv2.putText(frame, f"Distance: {distance_cm:.2f} cm", (cX - 20, cY - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    cv2.imshow('All Contours', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()