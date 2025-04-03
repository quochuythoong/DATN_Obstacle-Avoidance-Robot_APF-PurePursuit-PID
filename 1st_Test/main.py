###############################################################################
# LIBRARIES
###############################################################################
import cv2
import aruco_obstacle_detection as detection
import utils
import client_control as client
import numpy as np
from pure_pursuit import pure_pursuit_main, disable_pure_pursuit, enable_pure_pursuit
from Basic_APF import apf_path_planning
from Predictive_APF import predictive_path 
from utils import frame_height, kp, ki, kd

###############################################################################
# GLOBAL VARIABLES
###############################################################################
detection_active = False     # Flag to enable detection
coordinates_ready = False    # Flag to show PLAN_PATH button
predictive_APF_enable = True # Flag to enable Predictive APF
aruco_coordinates = None     # Detected ArUco marker coordinates
obstacle_coordinates = None  # Detected obstacle coordinates
goal_set_points = []         # Selected goal points
path_planning_enable = False # Flag to plan path
run_robot = False            # Flag to run the robot
pure_pursuit_enable = False  # Flag to enable Pure Pursuit
global_path = []             # Global path for Pure Pursuit
global_path_plot = None      # Global path for plotting (hold on)
interp_points_ellipse = []   # Global ellipse points
global_ellipse_plot = None   # Global ellipse for plotting (hold on)
aruco_path = []              # Global ArUco path
deviation_threshold = 40     # minimum deviation (perpendicular distance) required to create a temporary goal

###############################################################################
# Flag to control client
###############################################################################
flag_client_control = True

###############################################################################
# ARUCO SETUP
###############################################################################
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
parameters = cv2.aruco.DetectorParameters()  

###############################################################################
# BUTTON POSITIONS
###############################################################################
START_BUTTON_POS     = (5, 10, 65, 30)      # Green Start button
RESET_BUTTON_POS     = (75, 10, 135, 30)    # Red Reset button
CLEAR_BUTTON_POS     = (75, 35, 135, 55)    # Blue Clear button
PLAN_PATH_BUTTON_POS = (5, 35, 65, 55)      # Yellow PLAN_PATH button (Initially hidden)
APF_PAPF_BUTTON_POS  = (5, 60, 135, 80)     # Green/Red APF_PAPF button (toggle APF_PAPF flag)
RUN_BUTTON_POS       = (145, 10, 205, 80)   # Orange RUN button (appears after path planning)

# Position text directly below APF_PAPF button
flag_text_x = APF_PAPF_BUTTON_POS[0] + 10  # Align with the left side of the button
flag_text_y = APF_PAPF_BUTTON_POS[3] + 20  # Slightly below the bottom edge

###############################################################################
# FRAME INTERACTION 
###############################################################################
def mouse_callback(event, x, y, flags, param):
    global detection_active, coordinates_ready, aruco_coordinates, obstacle_coordinates, goal_set_points, path_planning_enable, global_path 
    global predictive_APF_enable, run_robot, pure_pursuit_enable, global_path_plot, global_ellipse_plot, aruco_path

    if event == cv2.EVENT_LBUTTONDOWN:
        # Start detection (capture an image, process, and return coordinates)
        if START_BUTTON_POS[0] <= x <= START_BUTTON_POS[2] and START_BUTTON_POS[1] <= y <= START_BUTTON_POS[3]:
            detection_active = True

        # Reset all processes
        elif RESET_BUTTON_POS[0] <= x <= RESET_BUTTON_POS[2] and RESET_BUTTON_POS[1] <= y <= RESET_BUTTON_POS[3]:
            # Reset various flags and states
            detection_active = False        # Disable obstacle detection
            coordinates_ready = False       # Clear coordinate readiness status

            # Reset key coordinate variables
            aruco_coordinates = None        # Clear ArUco marker coordinates
            obstacle_coordinates = None     # Clear obstacle coordinates

            # Clear stored interpolation and path planning data
            interp_points_ellipse.clear()   # Clear interpolated points for the ellipse
            global_ellipse_plot.clear()     # Clear global ellipse plot data
            goal_set_points.clear()         # Clear goal points
            global_path.clear()             # Clear global planned path

            # Reset path and motion control states
            path_planning_enable = False    # Disable path planning
            run_robot = False               # Stop robot execution
            global_path_plot = None         # Clear plotted path
            aruco_path = None               # Clear ArUco-based path
            pure_pursuit_enable = False     # Disable Pure Pursuit mode
            disable_pure_pursuit()          # Call function to disable Pure Pursuit

            # Stop the robot if controlled by a client
            if flag_client_control:
                client.ena_PID(0)           # Disable PID control
                client.send_params(0, 0)    # Stop the robot's movement

            # Print confirmation message
            print("RESET pressed")

        # Clear button
        elif CLEAR_BUTTON_POS[0] <= x <= CLEAR_BUTTON_POS[2] and CLEAR_BUTTON_POS[1] <= y <= CLEAR_BUTTON_POS[3]:
            print("Cleared all selections.")
            goal_set_points.clear()

        # PLAN_PATH button (Does nothing for now)
        elif coordinates_ready and PLAN_PATH_BUTTON_POS[0] <= x <= PLAN_PATH_BUTTON_POS[2] and PLAN_PATH_BUTTON_POS[1] <= y <= PLAN_PATH_BUTTON_POS[3]:
            print("Planning path...")
            run_robot = True
            path_planning_enable = True

        elif run_robot and RUN_BUTTON_POS[0] <= x <= RUN_BUTTON_POS[2] and RUN_BUTTON_POS[1] <= y <= RUN_BUTTON_POS[3]:
            print("Running the robot...")
            if flag_client_control:
                client.ena_PID(1) # Enable PID
            pure_pursuit_enable = True
            enable_pure_pursuit()

        # Toggle button (flag_predictive_APF) to enable / disable Predictive_APF
        elif APF_PAPF_BUTTON_POS[0] <= x <= APF_PAPF_BUTTON_POS[2] and APF_PAPF_BUTTON_POS[1] <= y <= APF_PAPF_BUTTON_POS[3]:
            predictive_APF_enable = not predictive_APF_enable  # Toggle predictive_APF_enable

        # Set goal point
        else:
            goal_set_points = [x, frame_height - y]
            print("Selected point:", goal_set_points)

# Function to align center for 'Text'
def draw_text_centered(frame, text, rect, font_scale, thickness=1, color=(255, 255, 255)):
    """ Draw text centered within a rectangle """
    text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)[0]
    text_x = rect[0] + (rect[2] - rect[0] - text_size[0]) // 2
    text_y = rect[1] + (rect[3] - rect[1] + text_size[1]) // 2
    cv2.putText(frame, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness)

###############################################################################
# GUI
###############################################################################
def draw_overlay(frame):
    """ Draw buttons and overlay with centered text """
    global global_path_plot, global_ellipse_plot
    
    font_scale = 0.5

    # Start Button (Green)
    cv2.rectangle(frame, START_BUTTON_POS[:2], START_BUTTON_POS[2:], (0, 255, 0), -1)
    draw_text_centered(frame, "START", START_BUTTON_POS, font_scale, color=(0, 0, 0))

    # Reset Button (Red)
    cv2.rectangle(frame, RESET_BUTTON_POS[:2], RESET_BUTTON_POS[2:], (0, 0, 255), -1)
    draw_text_centered(frame, "RESET", RESET_BUTTON_POS, font_scale, color=(255, 255, 255))

    # Clear Button (Blue)
    cv2.rectangle(frame, CLEAR_BUTTON_POS[:2], CLEAR_BUTTON_POS[2:], (255, 0, 0), -1)
    draw_text_centered(frame, "CLEAR", CLEAR_BUTTON_POS, font_scale, color=(255, 255, 255))

    # PLAN_PATH Button (Yellow) (Only if coordinates are ready)
    if coordinates_ready:
        cv2.rectangle(frame, PLAN_PATH_BUTTON_POS[:2], PLAN_PATH_BUTTON_POS[2:], (0, 255, 255), -1)
        draw_text_centered(frame, "PLAN", PLAN_PATH_BUTTON_POS, font_scale, color=(0, 0, 0))

    # RUN Button (Orange) (Only if path_planning_enable = TRUE, Path planning is done)
    if run_robot:
        cv2.rectangle(frame, RUN_BUTTON_POS[:2], RUN_BUTTON_POS[2:], (0, 165, 255), -1)
        draw_text_centered(frame, "RUN", RUN_BUTTON_POS, font_scale, color=(0, 0, 0))

    # APF-PAPF Toggle Button
    if predictive_APF_enable:
        button_color = (0, 200, 100)  # Green for PAPF   
    else: 
        button_color = (200, 50, 50)  # Red for APF
    cv2.rectangle(frame, APF_PAPF_BUTTON_POS[:2], APF_PAPF_BUTTON_POS[2:], button_color, -1)
    draw_text_centered(frame, "APF-PAPF", APF_PAPF_BUTTON_POS, font_scale, color=(255, 255, 255))

    # Display Flag Status (APF-PAPF)
    flag_status = "PAPF Enabled" if predictive_APF_enable else "APF Enabled"
    cv2.putText(frame, flag_status, 
                (flag_text_x, flag_text_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0) if predictive_APF_enable else (0, 0, 255), 2)

    # Draw the set points as red dots
    if goal_set_points:
        cv2.circle(frame, (goal_set_points[0], frame_height - goal_set_points[1]), 5, (0, 0, 255), -1)

    # Draw ellipse boundings
    if global_ellipse_plot is not None:
        for ellipse in global_ellipse_plot:
            for point in ellipse:
                x_ellipse, y_ellipse = point[0], point[1]
                cv2.circle(frame, (x_ellipse, y_ellipse), 1, (255, 255, 0), -1)     # Pastel-blue color

    # Draw the stored path as red dots (hold  on)
    if global_path_plot is not None:
        for point in global_path_plot:
            x_global_path, y__global_path = int(point[0]), int(frame_height - point[1])
            cv2.circle(frame, (x_global_path, y__global_path), 1, (0, 0, 255), -1)  # Red color
    
    return frame

###############################################################################
# GLOBAL PATH PLANNING
###############################################################################
def execute_path_planning(aruco_coordinates, obstacle_coordinates, goal_set_points, frame):
    global predictive_APF_enable, global_path, deviation_threshold, global_path_plot
    
    # Perform path planning (Basic APF)
    original_path = apf_path_planning(aruco_coordinates, goal_set_points, obstacle_coordinates)
    original_path.insert(0, aruco_coordinates)  # Insert the ArUco marker as the starting point
    original_path = utils.interpolate_waypoints(original_path)

    # Perform Advanced path planning (Predictive APF)
    if predictive_APF_enable:
        # Plan the Predictive Path
        pred_path = predictive_path(original_path, deviation_threshold)
        pred_path_final = utils.interpolate_waypoints(pred_path)

        # Save the path for plotting directly on the frame (hold on)
        global_path = pred_path_final
    elif not predictive_APF_enable:
        # Save the path for plotting directly on the frame (hold on)
        global_path = original_path

    global_path_plot = global_path

    # Plot the path directly on the frame for saving as an image
    for point in global_path:
        x_path, y_path = int(point[0]), int(frame_height - point[1]) # Convert to integers
        cv2.circle(frame, (x_path, y_path), 2, (0, 0, 255), -1)  # Red color

    # Save the final frame with the plotted path as a JPG image
    output_frame_filename = "2_Final_path.jpg"
    cv2.imwrite(output_frame_filename, frame)
    print(f"Final path saved as {output_frame_filename}")
    
###############################################################################
# MAIN 
###############################################################################
def main():
    global detection_active, coordinates_ready, aruco_coordinates, obstacle_coordinates, goal_set_points, path_planning_enable 
    global global_path, global_path_plot, global_ellipse_plot, aruco_path, kp, ki, kd
    
    # Initialize camera and window
    cap = detection.initialize_camera()
    cv2.namedWindow("Unified View", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("Unified View", mouse_callback)

    # send_PID to robot
    if flag_client_control:
        client.send_PID(kp, ki, kd)

    while True: # Loop until 'Reset' or 'q' is pressed

        frame, gray = detection.process_frame(cap)
        if frame is None:
            break

        # Enable RUN button if coordinates are ready
        if detection_active:
            # Perform detection once when START is pressed
            detection_active = False  # Disable after one capture
            aruco_coordinates, obstacle_coordinates, frame, end_point_arrow, angle, interp_points_ellipse = detection.detect_aruco_and_obstacles(frame, gray)
            global_ellipse_plot = interp_points_ellipse
            coordinates_ready = True
            print("Aruco Coordinates:", aruco_coordinates)
        
        # Perform Path Planning
        if coordinates_ready and path_planning_enable:
            execute_path_planning(aruco_coordinates, obstacle_coordinates, goal_set_points, frame)
            path_planning_enable = False

        # Perform Pure Pursuit
        if pure_pursuit_enable:
            corners, ids = detection.detect_aruco_markers_pure_pursuit(gray, aruco_dict, parameters)
            global_path, aruco_path = pure_pursuit_main(corners, global_path, frame)

        # Draw buttons and overlay information (GUI)
        draw_overlay(frame)

        # Display the frame
        cv2.imshow("Unified View", frame)

        # Press 'q' to exit
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
    
    # Stop the robot as the program exits
    if flag_client_control:
        client.ena_PID(0)
        client.send_params(0, 0)

    detection.release_camera(cap)
    
if __name__ == "__main__":
    main()
