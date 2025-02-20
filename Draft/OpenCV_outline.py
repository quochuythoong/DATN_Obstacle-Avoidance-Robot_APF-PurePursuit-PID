import cv2
import numpy as np
import sys
import matplotlib.pyplot as plt

def interpolate_waypoints(waypoints, step_distance=1.0):
    interpolated_points = []
    previous_point = None

    for i in range(len(waypoints) - 1):
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

    last_point = (int(round(waypoints[-1][0])), int(round(waypoints[-1][1])))
    if last_point != previous_point:
        interpolated_points.append(last_point)

    return interpolated_points

def main():
    image_path = "Picture3.jpg"  # Change this if necessary
    image = cv2.imread(image_path)
    if image is None:
        print("Error: Could not load image. Check the path and try again.")
        sys.exit(1)
    else:
        print("Image loaded successfully with shape:", image.shape)

    # Convert image to grayscale and blur
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Use Canny edge detection to extract edges
    edges = cv2.Canny(blurred, 50, 150)
    
    # --- Remove ArUco marker from the edges ---
    # Detect ArUco markers in the grayscale image
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    
    # If an ArUco marker is detected, fill its region in the edges image with black (0)
    if marker_ids is not None:
        # Create a mask with the same size as the edges image
        mask = np.zeros_like(edges)
        for marker in marker_corners:
            pts = marker.reshape((-1, 1, 2)).astype(np.int32)
            cv2.fillPoly(mask, [pts], 255)
        # Dilate the mask to extend the filled region (e.g., by 5 pixels)
        kernel = np.ones((5, 5), np.uint8)
        dilated_mask = cv2.dilate(mask, kernel, iterations=1)
        # Set the dilated region in the edges image to black
        edges[dilated_mask == 255] = 0
    # -------------------------------------------
    
    # Find contours from the modified edges image
    contours, hierarchy = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Prepare a Matplotlib figure to display the edge detection image
    plt.figure(figsize=(10, 10))
    plt.imshow(edges, cmap='gray')
    
    # Iterate over each contour, interpolate between points, and plot them
    for idx, cnt in enumerate(contours):
        # Convert contour points into a list of (x, y) tuples
        coords = cnt.reshape(-1, 2).tolist()
        print(f"\nContour {idx} has {len(coords)} original points:")
        for pt in coords:
            print(pt)
        
        # Interpolate along the contour's points using the provided function
        interp_points = interpolate_waypoints(coords, step_distance=1.0)
        print(f"\nContour {idx} interpolated points (total {len(interp_points)}):")
        for pt in interp_points:
            print(pt)
        
        # Plot the interpolated points on the edge detection image
        interp_points_array = np.array(interp_points)
        plt.scatter(interp_points_array[:, 0], interp_points_array[:, 1],
                    s=5, label=f"Contour {idx} interp")
    
    plt.title("Edge Detection with Interpolated Coordinates (ArUco Removed)")
    plt.legend()
    plt.axis("off")
    plt.show()

if __name__ == "__main__":
    main()
