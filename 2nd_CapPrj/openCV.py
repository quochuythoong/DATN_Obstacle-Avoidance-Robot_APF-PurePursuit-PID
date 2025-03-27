import cv2
import numpy as np

# Initialize OpenCV components and shared variables
def initialize_camera():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    return cap

def initialize_window(window_name, mouse_callback, callback_params):
    #cv2.namedWindow(window_name)
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1920, 1080)
    cv2.setMouseCallback(window_name, mouse_callback, callback_params)

def process_frame(cap):
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture image")
        return None
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return frame, gray

def detect_aruco_markers(gray, aruco_dict, parameters):
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    return corners, ids

def draw_detected_markers(frame, corners, ids):
    return cv2.aruco.drawDetectedMarkers(frame, corners, ids)

def draw_center_and_orientation(frame, corners, frame_height, frame_width):
    for i, corner in enumerate(corners):
        points = corner[0]
        
        # Calculate the center coordinates
        cx, cy = int(np.mean(points[:, 0])), frame_height - int(np.mean(points[:, 1]))
        center_coordinate = (cx, cy)
        
        # Calculate the orientation angle
        vec_x, vec_y = points[1][0] - points[0][0], points[0][1] - points[1][1]
        angle = np.arctan2(vec_y, vec_x) * 180 / np.pi

        # Draw the center and orientation arrow
        cv2.circle(frame, (cx, frame_height - cy), 5, (0, 255, 0), -1)  # Green circle at the center
        arrow_length = 50
        end_x = int(cx + arrow_length * np.cos(angle * np.pi / 180))
        end_y = int(cy + arrow_length * np.sin(angle * np.pi / 180))
        end_point_arrow = (end_x, end_y)
        cv2.arrowedLine(frame, (cx, frame_height - cy), (end_x, frame_height - end_y), (0, 255, 0), 2)  # Green arrow

        # Display coordinates and angle in the top right corner
        text_x_y = f"X: {cx}, Y: {cy}"
        text_angle = f"Angle: {angle:.2f} deg"
        
        # Set positions for the text in the top right corner
        top_right_x = frame_width - 200  # 200 pixels offset from the right edge
        top_right_y = 30  # Offset from the top edge for the first line of text
        line_spacing = 30  # Space between lines of text

        # Display the X, Y coordinates
        cv2.putText(frame, text_x_y, (top_right_x, top_right_y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Display the Angle
        cv2.putText(frame, text_angle, (top_right_x, top_right_y + line_spacing), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    return center_coordinate, end_point_arrow, angle

def draw_clicked_points(frame, clicked_points, frame_height):
    for i in range(len(clicked_points)):
        cv2.circle(frame, (clicked_points[i][0], frame_height - clicked_points[i][1]), 5, (0, 0, 255), -1)
        if i > 0:
            cv2.line(frame, (clicked_points[i - 1][0], frame_height - clicked_points[i - 1][1]), 
                     (clicked_points[i][0], frame_height - clicked_points[i][1]), (255, 0, 0), 2)

def display_text(frame, text, position):
    cv2.putText(frame, text, position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

def release_camera(cap):
    cap.release()
    cv2.destroyAllWindows()
