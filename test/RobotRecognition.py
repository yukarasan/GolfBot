import cv2
import numpy as np
import math
from flask import Flask
import threading

app = Flask(__name__)
angle = 0

def calculate_angle(center1, center2):
    x1, y1 = center1
    x2, y2 = center2
    return math.degrees(math.atan2(y2 - y1, x2 - x1))

@app.route("/")
def test():
    return "{:.2f}".format(angle)

def draw_line(image, start, end, color, thickness=2):
    height, width = image.shape[:2]

    x1, y1 = start
    x2, y2 = end

    if x1 == x2:  # Vertical line
        x1 = x2 = max(0, min(x1, width - 1))
        y1 = 0
        y2 = height - 1
    else:
        m = (y2 - y1) / (x2 - x1)  # Slope
        b = y1 - m * x1  # Intercept
        x1 = 0
        y1 = int(b)
        x2 = width - 1
        y2 = int(m * x2 + b)

    cv2.line(image, (x1, y1), (x2, y2), color, thickness)

def flask_server():
    app.run(port=8081)

def camera_thread():
    cap = cv2.VideoCapture(1)
    global angle

    while True:
        ret, frame = cap.read()
        frame = cv2.flip(frame, 1)
        if not ret:
            break

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_pink = np.array([150, 50, 50])
        upper_pink = np.array([180, 255, 255])

        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])

        blue_mask = cv2.inRange(hsv_frame, lower_pink, upper_pink)
        green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)

        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        blue_contour = max(blue_contours, key=cv2.contourArea) if blue_contours else None
        green_contour = max(green_contours, key=cv2.contourArea) if green_contours else None

        if blue_contour is not None and green_contour is not None:
            blue_moment = cv2.moments(blue_contour)
            green_moment = cv2.moments(green_contour)

            if blue_moment["m00"] != 0 and green_moment["m00"] != 0:
                blue_center = (int(blue_moment["m10"] / blue_moment["m00"]), int(blue_moment["m01"] / blue_moment["m00"]))
                green_center = (int(green_moment["m10"] / green_moment["m00"]), int(green_moment["m01"] / green_moment["m00"]))
                angle = new_angle = calculate_angle(blue_center, green_center)
                draw_line(frame, blue_center, green_center, (0, 255, 0), thickness=2)

                cv2.putText(frame, "Angle: {:.2f}".format(angle), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("Rectangles and Angle Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

flask_thread = threading.Thread(target=flask_server)
flask_thread.start()

camera_thread()
