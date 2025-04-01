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

# def draw_center_and_orientation(frame, corners, frame_height, frame_width, Adaptive_lookahead_pixels):
#     for i, corner in enumerate(corners):
#         points = corner[0]
        
#         # Calculate the center coordinates
#         cx, cy = int(np.mean(points[:, 0])), frame_height - int(np.mean(points[:, 1]))
#         center_coordinate = (cx, cy)
        
#         # Calculate the orientation angle
#         vec_x, vec_y = points[1][0] - points[0][0], points[0][1] - points[1][1]
#         angle = np.arctan2(vec_y, vec_x) * 180 / np.pi

#         # Draw the center and orientation arrow
#         cv2.circle(frame, (cx, frame_height - cy), 5, (0, 255, 0), -1)  # Green circle at the center
#         arrow_length = 50
#         end_x = int(cx + arrow_length * np.cos(angle * np.pi / 180))
#         end_y = int(cy + arrow_length * np.sin(angle * np.pi / 180))
#         end_point_arrow = (end_x, end_y)
#         cv2.arrowedLine(frame, (cx, frame_height - cy), (end_x, frame_height - end_y), (0, 255, 0), 2)  # Green arrow

#         # Display coordinates and angle in the top right corner
#         text_x_y = f"X: {cx}, Y: {cy}"
#         text_angle = f"Angle: {angle:.2f} deg"

#         # Display lookahead distance
#         text_x_y_lookahead = f"Lookahead: {Adaptive_lookahead_pixels:.2f} pixels"
        
#         # Set positions for the text in the top right corner
#         top_right_x = frame_width - 200  # 200 pixels offset from the right edge
#         top_right_y = 30  # Offset from the top edge for the first line of text
#         line_spacing = 30  # Space between lines of text

#         # Display the X, Y coordinates
#         cv2.putText(frame, text_x_y, (top_right_x, top_right_y), 
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

#         # Display the Angle
#         cv2.putText(frame, text_angle, (top_right_x, top_right_y + line_spacing), 
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
#         # Display the Lookahead distance
#         cv2.putText(frame, text_x_y_lookahead, (top_right_x, top_right_y + 2 * line_spacing), 
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

#     return center_coordinate, end_point_arrow, angle




def calculate_center_and_orientation(corners, frame_height):
    """
    Calculate the center coordinates and orientation angle from the detected ArUco corners.
    
    Parameters:
      corners: Detected marker corners.
      frame_height: The height of the frame (to adjust coordinate system).
      
    Returns:
      center_coordinate: (cx, cy) tuple.
      angle: Orientation angle in degrees.
      end_point_arrow: The computed endpoint of the orientation arrow.
    """
    # Process only the first marker (or extend to process multiple markers if needed)
    points = corners[0][0]
    
    # Calculate center coordinates
    cx = int(np.mean(points[:, 0]))
    cy = frame_height - int(np.mean(points[:, 1]))
    center_coordinate = (cx, cy)
    
    # Calculate orientation angle using top-left and top-right corners
    vec_x = points[1][0] - points[0][0]
    vec_y = points[0][1] - points[1][1]
    angle = np.arctan2(vec_y, vec_x) * 180 / np.pi
    
    # Calculate end of orientation arrow
    arrow_length = 50
    end_x = int(cx + arrow_length * np.cos(angle * np.pi / 180))
    end_y = int(cy + arrow_length * np.sin(angle * np.pi / 180))
    end_point_arrow = (end_x, end_y)
    
    return center_coordinate, end_point_arrow, angle

def draw_center_and_orientation_display(frame, center_coordinate, angle, end_point_arrow, Adaptive_lookahead_pixels, frame_width, frame_height):
    """
    Draw the center, orientation arrow, and display text for coordinates, angle, and lookahead distance.
    
    Parameters:
      frame: The image frame.
      center_coordinate: (cx, cy) tuple.
      angle: Orientation angle in degrees.
      end_point_arrow: Endpoint of the orientation arrow.
      Adaptive_lookahead_pixels: Lookahead distance in pixels.
      frame_width: Width of the frame.
      frame_height: Height of the frame.
      
    Returns:
      The modified frame.
    """
    # Draw the center and orientation arrow
    cv2.circle(frame, (center_coordinate[0], frame_height - center_coordinate[1]), 5, (0, 255, 0), -1)
    cv2.arrowedLine(frame, (center_coordinate[0], frame_height - center_coordinate[1]), 
                    (end_point_arrow[0], frame_height - end_point_arrow[1]), (0, 255, 0), 2)
    
    # Prepare text information
    text_x_y = f"X: {center_coordinate[0]}, Y: {center_coordinate[1]}"
    text_angle = f"Angle: {angle:.2f} deg"
    text_lookahead = f"Lookahead: {Adaptive_lookahead_pixels:.2f} px"
    
    # Set text positions
    top_right_x = frame_width - 200  # 200 pixels offset from right edge
    top_right_y = 30                # Top offset
    line_spacing = 30               # Space between lines
    
    # Display text
    cv2.putText(frame, text_x_y, (top_right_x, top_right_y), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv2.putText(frame, text_angle, (top_right_x, top_right_y + line_spacing), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv2.putText(frame, text_lookahead, (top_right_x, top_right_y + 2 * line_spacing), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)









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