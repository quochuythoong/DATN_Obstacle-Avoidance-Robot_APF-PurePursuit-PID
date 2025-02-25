# main.py
import cv2
import numpy as np
import aruco_obstacle_detection as detection
from apf_1st_implement import apf_path_planning
from utils import frame_height
import matplotlib.pyplot as plt

# Global variables
detection_active = False  # Flag to enable detection
coordinates_ready = False  # Flag to show RUN button
aruco_coordinates = None
obstacle_coordinates = None
goal_set_points = []
final_goal = None

# Button positions
START_BUTTON_POS = (10, 10, 150, 60)  # Green Start button
RESET_BUTTON_POS = (170, 10, 310, 60)  # Red Reset button
CLEAR_BUTTON_POS = (170, 70, 310, 120)  # Blue Clear button
RUN_BUTTON_POS = (10, 70, 150, 120)  # Yellow Run button (Initially hidden)

def mouse_callback(event, x, y, flags, param):
    global detection_active, coordinates_ready, aruco_coordinates, obstacle_coordinates, goal_set_points
    if event == cv2.EVENT_LBUTTONDOWN:
        # Start detection (capture an image, process, and return coordinates)
        if START_BUTTON_POS[0] <= x <= START_BUTTON_POS[2] and START_BUTTON_POS[1] <= y <= START_BUTTON_POS[3]:
            detection_active = True
            coordinates_ready = False
            if goal_set_points:
                final_goal = tuple(goal_set_points)
                print("Final goal confirmed:", final_goal)
        # Reset all processes
        elif RESET_BUTTON_POS[0] <= x <= RESET_BUTTON_POS[2] and RESET_BUTTON_POS[1] <= y <= RESET_BUTTON_POS[3]:
            detection_active = False
            coordinates_ready = False
            aruco_coordinates = None
            obstacle_coordinates = None
        # Clear button
        elif CLEAR_BUTTON_POS[0] <= x <= CLEAR_BUTTON_POS[2] and CLEAR_BUTTON_POS[1] <= y <= CLEAR_BUTTON_POS[3]:
            goal_set_points.clear()
            final_goal = None
            print("Cleared all selections.")
        # Run button (Does nothing for now)
        elif coordinates_ready and RUN_BUTTON_POS[0] <= x <= RUN_BUTTON_POS[2] and RUN_BUTTON_POS[1] <= y <= RUN_BUTTON_POS[3]:
            print("RUN button pressed.")
        else:
            goal_set_points.append((x, frame_height - y))
            print("Selected point:", goal_set_points[-1])

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
    for pt in goal_set_points:
        cv2.circle(frame, (pt[0], frame_height - pt[1]), 10, (0, 0, 255), -1)
    # If a final goal is confirmed, draw each point separately
    if final_goal is not None:
        for pt in final_goal:
            cv2.circle(frame, (pt[0], frame_height - pt[1]), 15, (0, 0, 255), -1)

def main():
    global detection_active, coordinates_ready, aruco_coordinates, obstacle_coordinates
    
    cap = detection.initialize_camera()
    cv2.namedWindow("Unified View", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("Unified View", mouse_callback)
    
    while True:
        frame, gray = detection.process_frame(cap)
        if frame is None:
            break
        
        if detection_active:
            # Perform detection once when START is pressed
            detection_active = False  # Disable after one capture
            aruco_coordinates, obstacle_coordinates, processed_frame = detection.detect_aruco_and_obstacles(frame, gray)
            coordinates_ready = True
            print("Aruco Coordinates:", aruco_coordinates)
            # print("Obstacle Coordinates:", obstacle_coordinates)
        
        draw_overlay(frame)
        cv2.imshow("Unified View", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
    print(obstacle_coordinates)
    path, potential_values = apf_path_planning(aruco_coordinates[0], goal_set_points, obstacle_coordinates)
    
    # Plot the results
    plt.figure(figsize=(6,11))
    plt.plot(path[:, 0], path[:, 1], "b.-", label="Smoothed Path")
    plt.scatter(*zip(*obstacle_coordinates), color="red", label="Obstacles")
    # plt.scatter(*(list(goal_set_points)), color="green", marker="x", s=100, label="Goal")
    # plt.scatter(*(list(aruco_coordinates[0])), color="black", marker="o", label="Start")
    plt.legend()
    plt.grid()
    plt.title("Artificial Potential Field Path Planning with Smoothing")
    # plt.show()

    # Plot the potential field evolution
    plt.figure()
    plt.plot(potential_values, "r-", label="Potential Field Value")
    plt.xlabel("Step")
    plt.ylabel("Potential")
    plt.title("Potential Field over Steps")
    plt.legend()
    plt.grid()
    plt.show()
    detection.release_camera(cap)
    
if __name__ == "__main__":
    main()
