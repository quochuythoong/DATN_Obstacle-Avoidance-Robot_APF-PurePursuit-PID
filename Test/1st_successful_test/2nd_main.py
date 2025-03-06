# main.py
import cv2
import aruco_obstacle_detection as detection
from pure_pursuit import pure_pursuit_main, disable_pure_pursuit, enable_pure_pursuit
from apf_1st_implement import apf_path_planning
from utils import frame_height
from client_control import send_params
import matplotlib.pyplot as plt

# Global variables
detection_active = False  # Flag to enable detection
coordinates_ready = False  # Flag to show RUN button
aruco_coordinates = None
obstacle_coordinates = None
goal_set_points = None
run_robot = False
current_frame = None # Variable to store the current frame

# Button positions
START_BUTTON_POS = (10, 10, 150, 60)  # Green Start button
RESET_BUTTON_POS = (170, 10, 310, 60)  # Red Reset button
CLEAR_BUTTON_POS = (170, 70, 310, 120)  # Blue Clear button
RUN_BUTTON_POS = (10, 70, 150, 120)  # Yellow Run button (Initially hidden)

def mouse_callback(event, x, y, flags, param):
    global detection_active, coordinates_ready, aruco_coordinates, obstacle_coordinates, goal_set_points, run_robot

    # Get the path from the callback parameter
    path = param 

    # Get the current frame from the global variable
    frame = current_frame

    if event == cv2.EVENT_LBUTTONDOWN:
        # Start detection (capture an image, process, and return coordinates)
        if START_BUTTON_POS[0] <= x <= START_BUTTON_POS[2] and START_BUTTON_POS[1] <= y <= START_BUTTON_POS[3]:
            detection_active = True
            enable_pure_pursuit()

        # Reset all processes
        elif RESET_BUTTON_POS[0] <= x <= RESET_BUTTON_POS[2] and RESET_BUTTON_POS[1] <= y <= RESET_BUTTON_POS[3]:
            detection_active = False
            coordinates_ready = False
            aruco_coordinates = None
            obstacle_coordinates = None
            path.clear()
            run_robot = False
            goal_set_points = None
            disable_pure_pursuit()
            # send_params(0, 0)  # Stop the robot

        # Clear button
        elif CLEAR_BUTTON_POS[0] <= x <= CLEAR_BUTTON_POS[2] and CLEAR_BUTTON_POS[1] <= y <= CLEAR_BUTTON_POS[3]:
            goal_set_points.clear()
            print("Cleared all selections.")

        # Run button (Does nothing for now)
        elif coordinates_ready and RUN_BUTTON_POS[0] <= x <= RUN_BUTTON_POS[2] and RUN_BUTTON_POS[1] <= y <= RUN_BUTTON_POS[3]:
            print("RUN")
            run_robot = True

        # Set goal point
        else:
            goal_set_points = [x, frame_height - y]
            print("Selected point:", goal_set_points)

def draw_overlay(frame):
    """ Draw buttons and information overlay """
    # Draw Start button (Green)
    cv2.rectangle(frame, START_BUTTON_POS[:2], START_BUTTON_POS[2:], (0, 255, 0), -1)
    cv2.putText(frame, "START", (START_BUTTON_POS[0] + 10, START_BUTTON_POS[1] + 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
    
    # Draw Reset button (Red)
    cv2.rectangle(frame, RESET_BUTTON_POS[:2], RESET_BUTTON_POS[2:], (0, 0, 255), -1)
    cv2.putText(frame, "RESET", (RESET_BUTTON_POS[0] + 10, RESET_BUTTON_POS[1] + 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    
    # Draw Clear button (Blue)
    cv2.rectangle(frame, CLEAR_BUTTON_POS[:2], CLEAR_BUTTON_POS[2:], (255, 0, 0), -1)
    cv2.putText(frame, "CLEAR", (CLEAR_BUTTON_POS[0] + 10, CLEAR_BUTTON_POS[1] + 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    
    # Draw Run button (Yellow) only if coordinates are ready
    if coordinates_ready:
        cv2.rectangle(frame, RUN_BUTTON_POS[:2], RUN_BUTTON_POS[2:], (0, 255, 255), -1)
        cv2.putText(frame, "RUN", (RUN_BUTTON_POS[0] + 10, RUN_BUTTON_POS[1] + 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
    
    # Draw the set points as red dots
    if goal_set_points:
        cv2.circle(frame, (goal_set_points[0], frame_height - goal_set_points[1]), 5, (0, 0, 255), -1)

# ''' --------------------------------- PLOTTING MATPLOTLIB FIGURES ---------------------------------

def execute_path_planning(aruco_coordinates, obstacle_coordinates, goal_set_points, frame):
    # Perform path planning
    path, potential_values = apf_path_planning(aruco_coordinates, goal_set_points, obstacle_coordinates)

    # Plot the path directly on the frame
    for point in path:
        x_path, y_path = int(point[0]), int(frame_height - point[1]) # Convert to integers
        cv2.circle(frame, (x_path, y_path), 3, (0, 0, 255), -1)  # Red color

    # Save the final frame with the plotted path as a JPG image
    output_frame_filename = "final_path.jpg"
    cv2.imwrite(output_frame_filename, frame)
    print(f"Final path saved as {output_frame_filename}")
'''
    # Plot the results
    plt.figure(figsize=(6,11))
    plt.plot(path[:, 0], path[:, 1], "b.-", label="Smoothed Path")
    plt.scatter(*zip(*obstacle_coordinates), color="red", label="Obstacles")
    # Ensure goal_set_points is not None before plotting
    if goal_set_points is not None and len(goal_set_points) > 0:
        plt.scatter(goal_set_points[0], goal_set_points[1], color="green", marker="x", s=100, label="Goal")
    # Ensure aruco_coordinates is not None and contains at least one detected marker before plotting
    if aruco_coordinates is not None and len(aruco_coordinates) > 0:
        plt.scatter(aruco_coordinates[0], aruco_coordinates[1], color="black", marker="o", label="Start")
    plt.grid()
    plt.xlabel("X-axis (pixels)")
    plt.ylabel("Y-axis (pixels)")
    plt.title("Artificial Potential Field Path Planning with Smoothing")
    plt.legend()

    # Plot the potential field evolution
    plt.figure()
    plt.plot(potential_values, "r-", label="Potential Field Value")
    plt.xlabel("Step")
    plt.ylabel("Potential")
    plt.title("Potential Field over Steps")
    plt.legend()
    plt.grid()

    # Display the Matplotlib figures (pauses execution until closed)
    plt.show()

# --------------------------------- PLOTTING MATPLOTLIB FIGURES --------------------------------- ''' 
    
def main():
    global detection_active, coordinates_ready, aruco_coordinates, obstacle_coordinates, goal_set_points, run_robot
    
    # Initialize camera and window
    cap = detection.initialize_camera()
    cv2.namedWindow("Unified View", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("Unified View", mouse_callback)
    
    while True: # Loop until 'Reset' or 'q' is pressed
        frame, gray = detection.process_frame(cap)
        if frame is None:
            break
        
        # Update the global current frame
        current_frame = frame.copy()

        # Enable RUN button if coordinates are ready
        if detection_active:
            # Perform detection once when START is pressed
            detection_active = False  # Disable after one capture
            aruco_coordinates, obstacle_coordinates, frame, end_point_arrow, angle = detection.detect_aruco_and_obstacles(frame, gray)
            coordinates_ready = True
            print("Aruco Coordinates:", aruco_coordinates)
        
        # Perform Path Planning and Pure Pursuit
        if coordinates_ready and run_robot:
        #   pure_pursuit_main(aruco_coordinates, obstacle_coordinates, goal_set_points, end_point_arrow)
            execute_path_planning(aruco_coordinates, obstacle_coordinates, goal_set_points, frame)
            run_robot = False

        # Draw buttons and overlay information (GUI)
        draw_overlay(frame)

        # Display the frame
        cv2.imshow("Unified View", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
    
    # send_params(0, 0)  # Stop the robot
    detection.release_camera(cap)

    
if __name__ == "__main__":
    main()
