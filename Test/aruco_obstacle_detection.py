# aruco_obstacle_detection.py
import cv2
import numpy as np
import cv2.aruco as aruco

def initialize_camera():
    cap = cv2.VideoCapture(0)  # Open camera
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    return cap

def process_frame(cap):
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture image")
        return None, None
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return frame, gray

def detect_aruco(frame, gray):
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters()
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    center = None
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        corner_points = corners[0][0]
        center = tuple(np.mean(corner_points, axis=0).astype(int))
        vector = corner_points[0] - corner_points[1]
        angle = np.arctan2(vector[1], vector[0])
        end_point = (int(center[0] + 50 * np.cos(angle)), int(center[1] + 50 * np.sin(angle)))
        cv2.arrowedLine(frame, center, end_point, (0, 255, 0), 2, tipLength=0.3)
    return corners, ids, center

def detect_obstacles(frame, gray):
    _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    obstacle_boxes = []
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        if w * h > 500:
            obstacle_boxes.append((x, y, x + w, y + h))
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
    return obstacle_boxes

def detect_aruco_and_obstacles(frame, gray):
    corners, ids, center = detect_aruco(frame, gray)
    obstacles = detect_obstacles(frame, gray)
    return corners, ids, center, obstacles

def release_camera(cap):
    cap.release()
    cv2.destroyAllWindows()
