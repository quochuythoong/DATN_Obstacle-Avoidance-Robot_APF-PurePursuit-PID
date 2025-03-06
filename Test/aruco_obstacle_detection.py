# aruco_obstacle_detection.py
import cv2
import numpy as np
import cv2.aruco as aruco
import matplotlib.pyplot as plt
from utils import frame_height

output_filename = "Processed_image.jpg"

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

def interpolate_waypoints(waypoints, step_distance=1.0):
    interpolated_points = []
    previous_point = None
    n = len(waypoints)

    if n < 2:
        return waypoints
    
    # Interpolate between consecutive points
    for i in range(n - 1):
        start = np.array(waypoints[i])
        end = np.array(waypoints[i + 1])
        distance = np.linalg.norm(end - start)
        if distance == 0:
            continue
        direction = (end - start) / distance
        num_steps = int(distance // step_distance) + 1

        for step in range(num_steps):
            interpolated_point = start + step * step_distance * direction
            rounded_point = (int(round(interpolated_point[0])), int(round(interpolated_point[1])))
            if rounded_point != previous_point:
                interpolated_points.append(rounded_point)
                previous_point = rounded_point

    # Interpolate between the last point and the first point to close the contour
    start = np.array(waypoints[-1])
    end = np.array(waypoints[0])
    distance = np.linalg.norm(end - start)
    if distance != 0:
        direction = (end - start) / distance
        num_steps = int(distance // step_distance) + 1
        for step in range(1, num_steps):  # start from 1 to avoid duplicating the last point
            interpolated_point = start + step * step_distance * direction
            rounded_point = (int(round(interpolated_point[0])), int(round(interpolated_point[1])))
            if rounded_point != previous_point:
                interpolated_points.append(rounded_point)
                previous_point = rounded_point

    return interpolated_points

# def save_coordinates_to_txt(file_name, aruco_coordinates, obstacle_coordinates):
#     with open(file_name, "w") as file:
#         file.write("Aruco Coordinates:\n")
#         for aruco in aruco_coordinates:
#             file.write(f"ID {aruco[0]}: (X: {aruco[1]}, Y: {aruco[2]})\n")

#         file.write("\nObstacle Coordinates:\n")
#         for i, obstacle in enumerate(obstacle_coordinates):
#             file.write(f"Obstacle {i + 1}:\n")
#             for point in obstacle:
#                 file.write(f"({point[0]}, {point[1]}) ")
#             file.write("\n")  # New line after each obstacle
        
#         # Write the full array of obstacle_coordinates in a readable format
#         file.write("\nFull Obstacle Coordinates (Array of Arrays):\n")
#         file.write("[\n")
#         for obstacle in obstacle_coordinates:
#             file.write(f"  {obstacle},\n")
#         file.write("]\n")

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
        kernel = np.ones((10, 10), np.uint8)
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
        interp_points = interpolate_waypoints(coords, step_distance=1.0)
        
        # Invert y-coordinates of detected obstacles
        inverted_interp_points = [(x, frame_height - y) for x, y in interp_points]

        # Append only the current contour's points
        obstacle_coordinates = obstacle_coordinates + inverted_interp_points
        
        # Plot the interpolated points on the edge detection image
        interp_points_array = np.array(interp_points)
        plt.scatter(interp_points_array[:, 0], interp_points_array[:, 1],
                    s=5, label=f"Contour {idx} interp")

    # Display the Matplotlib figure
    plt.title("Edge Detection with Interpolated Coordinates (ArUco Ignored)")
    plt.legend()
    plt.axis("off")
    plt.savefig(output_filename, bbox_inches='tight', pad_inches=0)

    print(f"Processed image saved as {output_filename}")

    # Save detected coordinates to a .txt file
    # save_coordinates_to_txt("Processed_image_data.txt", aruco_coordinates, obstacle_coordinates)

    return aruco_coordinates, obstacle_coordinates, frame, end_point_arrow, angle



