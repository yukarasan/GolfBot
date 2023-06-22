import cv2
import numpy as np

cap = cv2.VideoCapture(1)

red_lower = np.array([0, 100, 100])
red_upper = np.array([10, 255, 255])

black_lower = np.array([0, 0, 0])
black_upper = np.array([180, 255, 30])

real_width = 120
real_length = 180

scale_w = 1
scale_h = 1

while True:
    ret, frame = cap.read()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask_red = cv2.inRange(hsv, red_lower, red_upper)
    kernel = np.ones((5, 5), np.uint8)
    mask_red = cv2.erode(mask_red, kernel, iterations=1)
    mask_red = cv2.dilate(mask_red, kernel, iterations=1)
    blur_red = cv2.blur(mask_red, (14, 14))

    contours_red, hierarchy_red = cv2.findContours(blur_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    if contours_red:
        max_contour_red = max(contours_red, key=cv2.contourArea)

        x, y, w, h = cv2.boundingRect(max_contour_red)

        scale_w = real_width / w
        scale_h = real_length / h

        goal1 = int(180 / 2)
        goal2 = int(180 / 2)

        cv2.circle(frame, goal1, 5, (0, 255, 255), -1)  # goal on one side
        cv2.circle(frame, goal2, 5, (0, 255, 255), -1)  # goal on the other side

        mask_black = cv2.inRange(hsv, black_lower, black_upper)
        mask_black = cv2.erode(mask_black, kernel, iterations=1)
        mask_black = cv2.dilate(mask_black, kernel, iterations=1)

        mask_rect = np.zeros_like(mask_black)
        cv2.rectangle(mask_rect, (x, y), (x + w, y + h), (255, 255, 255), -1)

        mask_black = cv2.bitwise_and(mask_black, mask_rect)
        blur_black = cv2.blur(mask_black, (14, 14))

        contours_black, hierarchy_black = cv2.findContours(blur_black, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        cv2.drawContours(frame, contours_black, -1, (255, 0, 0), 3)

    cv2.drawContours(frame, contours_red, -1, (0, 255, 0), 3)

    cv2.imshow('Contours', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
