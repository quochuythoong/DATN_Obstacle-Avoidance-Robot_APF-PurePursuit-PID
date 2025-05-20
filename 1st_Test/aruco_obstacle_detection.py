###############################################################################
# LIBRARIES
###############################################################################
import cv2
import numpy as np
import cv2.aruco as aruco
import matplotlib.pyplot as plt
import utils
from utils import frame_height, frame_width

###############################################################################
# GLOBAL VARIABLES
###############################################################################
output_filename = "1_Processed_image.jpg"

###############################################################################
# CAMERA FUNCTIONS
###############################################################################
def initialize_camera():
    """ Initializes and returns the camera object """
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
    # cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)  
    # cap.set(cv2.CAP_PROP_EXPOSURE, -4) 
    # cap.set(cv2.CAP_PROP_BRIGHTNESS, 0) 
    return cap

def release_camera(cap):
    """ Releases the camera resource """
    cap.release()
    cv2.destroyAllWindows()

def process_frame(cap):
    """ Captures and processes a frame from the camera """
    ret, frame = cap.read()
    if not ret:
        return None, None
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return frame, gray

###############################################################################
# ELLIPSE BOUNDING FOR OBSTACLES
###############################################################################
def ellipse_bounding(points, expansion_factor=1.2, num_points=100):
    """
    Fits an ellipse to the given points and returns a list of points along the ellipse boundary.
    
    Parameters:
      points (array-like): A list or array of (x, y) points representing a contour.
      expansion_factor (float): Factor to scale the fitted ellipse's axes (1.0 = no expansion).
      num_points (int): Number of boundary points to generate along the ellipse.
    
    Returns:
      list: A list of (x, y) tuples along the ellipse boundary.
    """

    # Convert input to a NumPy array of type float32
    pts = np.array(points, dtype=np.float32)
    
    # If there are not enough points to fit an ellipse, return the original points
    if pts.shape[0] < 5:
        return [tuple(p) for p in pts]
    
    # Reshape to the format required by cv2.fitEllipse: (N, 1, 2)
    pts_reshaped = pts.reshape((-1, 1, 2))
    
    # Fit an ellipse using OpenCV
    ellipse = cv2.fitEllipse(pts_reshaped)
    center, axes, angle = ellipse  # center=(cx,cy), axes=(width, height), angle in degrees
    
    # Expand the axes by the expansion_factor
    a = (axes[0] * expansion_factor) / 2.0  # semi-major axis
    b = (axes[1] * expansion_factor) / 2.0  # semi-minor axis
    theta = np.deg2rad(angle)  # convert angle to radians
    
    # Generate points along the ellipse using the parametric equation\n    # x = a*cos(t), y = b*sin(t)\n    t in [0, 2*pi]\n    t = np.linspace(0, 2*pi, num_points)
    t = np.linspace(0, 2 * np.pi, num_points)
    x = a * np.cos(t)
    y = b * np.sin(t)
    
    # Rotate the ellipse points by theta using a rotation matrix
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta),  np.cos(theta)]])
    ellipse_points = np.dot(R, np.vstack((x, y))).T + center
    
    # Interpolate along the ellipse boundary
    interp_ellipse = utils.interpolate_waypoints_contour(ellipse_points)
    
    # Return as a list of interpolated (x, y) tuples
    return interp_ellipse

###############################################################################
# SMALL OBSTACLES FILTER
###############################################################################
def filter_small_obstacles(contours, min_points=10):
    """
    Filters out contours that have min_points or fewer points.
    
    Parameters:
      contours (list): List of contour arrays.
      min_points (int): Minimum number of points required to keep a contour.
    
    Returns:
      list: Filtered list of contours with more than min_points.
    """
    filtered_contours = []
    for cnt in contours:
        # cnt may have shape (N, 1, 2) or (N, 2)
        if cnt.shape[0] > min_points:
            filtered_contours.append(cnt)
    return filtered_contours

###############################################################################
# ARUCO & OBSTACLES DETECTION
###############################################################################
def detect_aruco_and_obstacles(frame, gray):
    """ Detects ArUco markers and obstacles in the frame """
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
    parameters = aruco.DetectorParameters()
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    aruco_coordinates = []
    if ids is not None:
        for i, corner in enumerate(corners):
            points = corner[0]

            # Calculate the center of the ArUco marker
            x, y = int(corner[0][:, 0].mean()), int(corner[0][:, 1].mean())
            aruco_coordinates = [x, frame_height - y]

            # Calculate the orientation angle
            vec_x, vec_y = points[1][0] - points[0][0], points[0][1] - points[1][1]
            angle = np.arctan2(vec_y, vec_x) * 180 / np.pi

            # Draw the center and orientation arrow
            cv2.circle(frame, (x, frame_height - y), 5, (0, 255, 0), -1)  # Green circle at the center
            arrow_length = 50
            end_x = int(x + arrow_length * np.cos(angle * np.pi / 180))
            end_y = int(y + arrow_length * np.sin(angle * np.pi / 180))
            end_point_arrow = (end_x, end_y)
            # cv2.arrowedLine(frame, (x, frame_height - y), (end_x, frame_height - end_y), (0, 255, 0), 2)  # Green arrow   
    
    # Detect obstacles (everything that isn't an ArUco marker)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)

    # If an ArUco marker is detected, fill its region in the edges image with black (0)
    if ids is not None:
        # Create a mask with the same size as the edges image
        mask = np.zeros_like(edges)
        for marker in corners:
            pts = marker.reshape((-1, 1, 2)).astype(np.int32)
            cv2.fillPoly(mask, [pts], 255)
        # Dilate the mask to extend the filled region (e.g., by 20 pixels)
        kernel = np.ones((20, 20), np.uint8)
        dilated_mask = cv2.dilate(mask, kernel, iterations=5)
        # Set the dilated region in the edges image to black
        edges[dilated_mask == 255] = 0
    # -------------------------------------------

    # Detect Contours
    contours, hierarchy = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Filter small contours
    small_contours_removed = filter_small_obstacles(contours, min_points=20)
    
    # Prepare a Matplotlib figure to display the edge detection image
    plt.figure(figsize=(10, 10))
    plt.imshow(edges, cmap='gray')

    obstacle_coordinates = []
    appended_interp_points_ellipse = []
    for idx, cnt in enumerate(small_contours_removed):
        # Convert contour points into a list of (x, y) tuples
        coords = cnt.reshape(-1, 2).tolist()
        
        # Interpolate along the contour's points using the provided function
        contour_interp_points = utils.interpolate_waypoints_contour(coords) # OpenCV axis (with downward y)

        # Ellipse bounding of detected obstacles
        ellipse_bounded_obstacles_show = ellipse_bounding(contour_interp_points)
        ellipse_bounded_obstacles_calculate = [(x, frame_height - y) for x, y in ellipse_bounded_obstacles_show]

        # Append only the current contour's points
        obstacle_coordinates = obstacle_coordinates + ellipse_bounded_obstacles_calculate
        
        # Plot the interpolated points on the edge detection image
        interp_points_array = np.array(contour_interp_points)
        plt.scatter(interp_points_array[:, 0], interp_points_array[:, 1],
                    s=5, color='blue', label=f"Contour {idx} interp")
        
        # Plot the interpolated points of the ellipse bounding
        interp_points_ellipse = np.array(ellipse_bounded_obstacles_show)
        plt.scatter(interp_points_ellipse[:, 0], interp_points_ellipse[:, 1],
                    s=5, color='red', label=f"Contour {idx} ellipse")
        
        appended_interp_points_ellipse.append(interp_points_ellipse.tolist())

    # Display the Matplotlib figure
    plt.title("Edge Detection with Interpolated Coordinates & Ellipse (ArUco Ignored)")
    # plt.legend()
    plt.axis("off")
    plt.savefig(output_filename, bbox_inches='tight', pad_inches=0)

    print(f"Processed image saved as {output_filename}")

    return aruco_coordinates, obstacle_coordinates, frame, end_point_arrow, angle, appended_interp_points_ellipse, small_contours_removed

###############################################################################
# PURE PURSUIT ADD-ON FUNCTIONS (LIVE TRACKING)
###############################################################################
def detect_aruco_markers_pure_pursuit(gray, aruco_dict, parameters):
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    return corners, ids

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
    top_right_x = frame_width - 200 # 200 pixels offset from right edge
    top_right_y = 30                # Top offset
    line_spacing = 30               # Space between lines
    
    # Display text
    cv2.putText(frame, text_x_y, (top_right_x, top_right_y), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv2.putText(frame, text_angle, (top_right_x, top_right_y + line_spacing), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv2.putText(frame, text_lookahead, (top_right_x, top_right_y + 2 * line_spacing), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

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

def aruco_path_plot(frame, center_coordinate, flag_end_waypoint, aruco_path_store):
    global frame_height

    aruco_path_store.append(center_coordinate)
    
    # Wait until the robot reaches the last waypoint, then interpolate and plot the path that the actual robot has taken
    if flag_end_waypoint:
        aruco_path_store = np.array(aruco_path_store, dtype=np.float64).reshape(-1, 2)
        
        # Interpolate waypoints
        aruco_path_store = utils.interpolate_waypoints(aruco_path_store, step_distance=1.0)
        
        # Draw the path on the frame
        for point in aruco_path_store:
            aruco_x_path, aruco_y_path = int(point[0]), int(frame_height - point[1]) # Convert to integers
            cv2.circle(frame, (aruco_x_path, aruco_y_path), 2, (0, 255, 0), -1)  # Green color
    
    return aruco_path_store