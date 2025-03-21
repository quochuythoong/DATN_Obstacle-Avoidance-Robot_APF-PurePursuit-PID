###############################################################################
# LIBRARIES
###############################################################################
import cv2
import numpy as np
import cv2.aruco as aruco
import matplotlib.pyplot as plt
import utils
from utils import frame_height

###############################################################################
# GLOBAL VARIABLES
###############################################################################
output_filename = "Processed_image.jpg"

###############################################################################
# CAMERA FUNCTIONS
###############################################################################
def initialize_camera():
    """ Initializes and returns the camera object """
    cap = cv2.VideoCapture(0)
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
# ARUCO & OBSTACLES DETECTION
###############################################################################
def detect_aruco_and_obstacles(frame, gray):
    """ Detects ArUco markers and obstacles in the frame """
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
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
        # Dilate the mask to extend the filled region (e.g., by 10 pixels)
        kernel = np.ones((30, 30), np.uint8)
        dilated_mask = cv2.dilate(mask, kernel, iterations=1)
        # Set the dilated region in the edges image to black
        edges[dilated_mask == 255] = 0
    # -------------------------------------------

    contours, hierarchy = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Prepare a Matplotlib figure to display the edge detection image
    plt.figure(figsize=(10, 10))
    plt.imshow(edges, cmap='gray')

    obstacle_coordinates = []
    for idx, cnt in enumerate(contours):
        # Convert contour points into a list of (x, y) tuples
        coords = cnt.reshape(-1, 2).tolist()
        
        # Interpolate along the contour's points using the provided function
        contour_interp_points = utils.interpolate_waypoints_contour(coords) # OpenCV axis (with downward y)
        
        # Invert y-coordinates of detected obstacles
        inverted_interp_points = [(x, frame_height - y) for x, y in contour_interp_points] # Upward y axis

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

    # Display the Matplotlib figure
    plt.title("Edge Detection with Interpolated Coordinates & Ellipse (ArUco Ignored)")
    # plt.legend()
    plt.axis("off")
    plt.savefig(output_filename, bbox_inches='tight', pad_inches=0)

    print(f"Processed image saved as {output_filename}")

    # Save detected coordinates to a .txt file
    # save_coordinates_to_txt("Processed_image_data.txt", aruco_coordinates, obstacle_coordinates)

    return aruco_coordinates, obstacle_coordinates, frame, end_point_arrow, angle, interp_points_ellipse

###############################################################################
# PURE PURSUIT ADD-ON FUNCTIONS (LIVE TRACKING)
###############################################################################
def detect_aruco_markers_pure_pursuit(gray, aruco_dict, parameters):
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    return corners, ids

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