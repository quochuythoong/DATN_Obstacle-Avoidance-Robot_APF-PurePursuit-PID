import cv2
import numpy as np
import cv2.aruco as aruco

# Global flag to control detection
detection_active = False

# Define button positions as (x1, y1, x2, y2)
START_BUTTON_POS = (10, 10, 150, 60)
RESET_BUTTON_POS = (170, 10, 310, 60)

def mouse_callback(event, x, y, flags, param):
    global detection_active
    if event == cv2.EVENT_LBUTTONDOWN:
        # Check if click is inside the Start button
        if START_BUTTON_POS[0] <= x <= START_BUTTON_POS[2] and START_BUTTON_POS[1] <= y <= START_BUTTON_POS[3]:
            detection_active = True
            # print("Start pressed: Detection enabled.")
        # Check if click is inside the Reset button
        elif RESET_BUTTON_POS[0] <= x <= RESET_BUTTON_POS[2] and RESET_BUTTON_POS[1] <= y <= RESET_BUTTON_POS[3]:
            detection_active = False
            # print("Reset pressed: Detection disabled.")

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

def draw_buttons(frame):
    # Draw Start button (Green)
    cv2.rectangle(frame, (START_BUTTON_POS[0], START_BUTTON_POS[1]),
                  (START_BUTTON_POS[2], START_BUTTON_POS[3]), (0, 255, 0), -1)
    cv2.putText(frame, "Start", (START_BUTTON_POS[0] + 10, START_BUTTON_POS[1] + 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
    # Draw Reset button (Red)
    cv2.rectangle(frame, (RESET_BUTTON_POS[0], RESET_BUTTON_POS[1]),
                  (RESET_BUTTON_POS[2], RESET_BUTTON_POS[3]), (0, 0, 255), -1)
    cv2.putText(frame, "Reset", (RESET_BUTTON_POS[0] + 10, RESET_BUTTON_POS[1] + 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

def detect_aruco(frame, gray):
    # Using the original ArUco API:
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters()  # Updated version
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    center = None

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        corner_points = corners[0][0]
        center = tuple(np.mean(corner_points, axis=0).astype(int))
        # Compute orientation based on the first two corners
        vector = corner_points[0] - corner_points[1]
        angle = np.arctan2(vector[1], vector[0])
        end_point = (int(center[0] + 50 * np.cos(angle)), int(center[1] + 50 * np.sin(angle)))
        cv2.arrowedLine(frame, center, end_point, (0, 255, 0), 2, tipLength=0.3)
    return corners, ids, center

def detect_obstacles(frame, gray):
    """
    Detect obstacles by thresholding the whole gray image.
    If ArUco markers are detected, their regions are masked out.
    Returns a list of bounding boxes for obstacles.
    """
    # Threshold entire gray image to get candidate obstacles
    _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
    
    # If ArUco markers are detected, mask out their regions
    # (Assuming they have been drawn on the frame and we have the corners available)
    # In this module we do not explicitly pass marker corners, so we simply proceed.
    # You may further refine this if you want to exclude marker regions.
    
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    obstacle_boxes = []
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        if w * h > 500:  # Filter out small noise
            obstacle_boxes.append((x, y, x + w, y + h))
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
    return obstacle_boxes

def detect_aruco_and_obstacles(frame, gray):
    """ Combines ArUco detection and obstacle detection. """
    corners, ids, center = detect_aruco(frame, gray)
    obstacles = detect_obstacles(frame, gray)
    return corners, ids, center, obstacles

def release_camera(cap):
    cap.release()
    cv2.destroyAllWindows()

def main():
    global detection_active
    cap = initialize_camera()
    cv2.namedWindow("Aruco and Obstacles Detection", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Aruco and Obstacles Detection", 1920, 1080)
    cv2.setMouseCallback("Aruco and Obstacles Detection", mouse_callback)
    
    aruco_coords = None
    obstacle_boxes = []
    
    while True:
        frame, gray = process_frame(cap)
        if frame is None:
            break
        
        draw_buttons(frame)
        
        if detection_active:
            corners, ids, center, obstacles = detect_aruco_and_obstacles(frame, gray)
            if center is not None:
                aruco_coords = center
            obstacle_boxes = obstacles
        else:
            aruco_coords = None
            obstacle_boxes = []
        
        cv2.imshow("Aruco and Obstacles Detection", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    release_camera(cap)
    print("Aruco Marker Coordinates:", aruco_coords)
    print("Obstacle Bounding Boxes:", obstacle_boxes)

if __name__ == "__main__":
    main()
