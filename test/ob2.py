import cv2
import numpy as np

def calculate_distance(pt1, pt2):
    return np.sqrt((pt1[0] - pt2[0]) ** 2 + (pt1[1] - pt2[1]) ** 2)

red_lower = np.array([0, 100, 100])
red_upper = np.array([10, 255, 255])

cap = cv2.VideoCapture(0)

kernel = np.ones((5, 5), np.uint8)

while True:
    ret, frame = cap.read()

    frame_height, frame_width = frame.shape[:2]

    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    image_blur = cv2.GaussianBlur(gray, (5, 5), 0)

    circles = cv2.HoughCircles(image_blur, cv2.HOUGH_GRADIENT, dp=1, minDist=30, param1=50, param2=20, minRadius=14,
                               maxRadius=17)
    if circles is not None:
        circles = np.round(circles[0, :]).astype(int)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 100])
        upper_white = np.array([179, 255, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        filtered_circles = []
        for (x, y, r) in circles:
            if mask[y, x] == 255:
                filtered_circles.append((x, y, r))

        for (x, y, r) in filtered_circles:
            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
            center = (int(x), int(y))

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask_red = cv2.inRange(hsv, red_lower, red_upper)
            mask_red = cv2.erode(mask_red, kernel, iterations=1)
            mask_red = cv2.dilate(mask_red, kernel, iterations=1)

            blur_red = cv2.blur(mask_red, (14, 14))
            contours_red, _ = cv2.findContours(blur_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
            cv2.drawContours(frame, contours_red, -1, (0, 0, 255), 2)

            min_distance = float('inf')
            closest_point = None

            for contour in contours_red:
                for point in contour:
                    distance = calculate_distance(center, (point[0][0], point[0][1]))
                    if distance < min_distance:
                        min_distance = distance
                        closest_point = point[0]

            if closest_point is not None:
                opposite_vector = np.array([closest_point[0] - center[0], closest_point[1] - center[1]])
                opposite_vector = opposite_vector.astype(float)
                opposite_vector /= np.linalg.norm(opposite_vector)

                wholeFrame = (frame_width // 2, frame_height // 2)

                theta = np.radians(20) 
                rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
                rotated_vector = np.dot(rotation_matrix, opposite_vector)
                rotated_vector *= -1
                cv2.circle(frame, (closest_point[0], closest_point[1]), 10, (0, 255, 0), 2)
                end_point = (int(center[0] + 100 * rotated_vector[0]), int(center[1] + 100 * rotated_vector[1]))

                cv2.arrowedLine(frame, center, end_point, (0, 0, 255), 2)

    cv2.imshow("Frame", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()