#main.py
import cv2
import numpy as np
import aruco_obstacle_detection as detection

# Global variables for goal selection
goal_set_points = []
final_goal = None
detection_active = False  # Detection initially disabled

# Define button positions (relative to the single frame)
START_BUTTON_POS = (10, 10, 150, 60)  # Green Start button
RESET_BUTTON_POS = (170, 10, 310, 60)  # Red Reset button
CONFIRM_BUTTON_POS = (10, 70, 150, 120)  # Yellow Confirm button
CLEAR_BUTTON_POS = (170, 70, 310, 120)  # Blue Clear button

def mouse_callback(event, x, y, flags, param):
    global goal_set_points, final_goal, detection_active
    if event == cv2.EVENT_LBUTTONDOWN:
        # Check if click is inside the Start button area
        if START_BUTTON_POS[0] <= x <= START_BUTTON_POS[2] and START_BUTTON_POS[1] <= y <= START_BUTTON_POS[3]:
            detection_active = True
        # Check if click is inside the Reset button area
        elif RESET_BUTTON_POS[0] <= x <= RESET_BUTTON_POS[2] and RESET_BUTTON_POS[1] <= y <= RESET_BUTTON_POS[3]:
            detection_active = False
        # Check if click is inside the Confirm button area
        elif CONFIRM_BUTTON_POS[0] <= x <= CONFIRM_BUTTON_POS[2] and CONFIRM_BUTTON_POS[1] <= y <= CONFIRM_BUTTON_POS[3]:
            if goal_set_points:
                final_goal = tuple(goal_set_points)
                print("Final goal confirmed:", final_goal)
        # Check if click is inside the Clear button area
        elif CLEAR_BUTTON_POS[0] <= x <= CLEAR_BUTTON_POS[2] and CLEAR_BUTTON_POS[1] <= y <= CLEAR_BUTTON_POS[3]:
            goal_set_points.clear()
            final_goal = None
            print("Goal selection cleared.")
        else:
            goal_set_points.append((x, y))
            print("Goal point added:", (x, y))

def draw_overlay(frame):
    """
    Draws buttons and goal points directly on the detection frame.
    """
    # Draw Start button (Green)
    cv2.rectangle(frame, (START_BUTTON_POS[0], START_BUTTON_POS[1]),
                  (START_BUTTON_POS[2], START_BUTTON_POS[3]), (0, 255, 0), -1)
    cv2.putText(frame, "Start", (START_BUTTON_POS[0] + 10, START_BUTTON_POS[1] + 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
    # Draw Reset button (Red)
    cv2.rectangle(frame, (RESET_BUTTON_POS[0], RESET_BUTTON_POS[1]),
                  (RESET_BUTTON_POS[2], RESET_BUTTON_POS[3]), (0, 0, 255), -1)
    cv2.putText(frame, "Reset", (RESET_BUTTON_POS[0] + 10, RESET_BUTTON_POS[1] + 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    # Draw Confirm button (Yellow)
    cv2.rectangle(frame, (CONFIRM_BUTTON_POS[0], CONFIRM_BUTTON_POS[1]),
                  (CONFIRM_BUTTON_POS[2], CONFIRM_BUTTON_POS[3]), (0, 255, 255), -1)
    cv2.putText(frame, "Confirm", (CONFIRM_BUTTON_POS[0] + 5, CONFIRM_BUTTON_POS[1] + 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
    # Draw Clear button (Blue)
    cv2.rectangle(frame, (CLEAR_BUTTON_POS[0], CLEAR_BUTTON_POS[1]),
                  (CLEAR_BUTTON_POS[2], CLEAR_BUTTON_POS[3]), (255, 0, 0), -1)
    cv2.putText(frame, "Clear", (CLEAR_BUTTON_POS[0] + 5, CLEAR_BUTTON_POS[1] + 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    # Draw the set points as red dots
    for pt in goal_set_points:
        cv2.circle(frame, pt, 5, (0, 0, 255), -1)
    # If a final goal is confirmed, draw each point separately
    if final_goal is not None:
        for pt in final_goal:
            cv2.circle(frame, pt, 10, (0, 0, 255), -1)

def main():
    global final_goal
    cap = detection.initialize_camera()
    
    cv2.namedWindow("Unified View", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("Unified View", mouse_callback)
    
    while True:
        frame, gray = detection.process_frame(cap)
        if frame is None:
            break
        
        if detection_active:
            corners, ids, center, obstacles = detection.detect_aruco_and_obstacles(frame, gray)
        else:
            obstacles = []
        
        draw_overlay(frame)
        
        cv2.imshow("Unified View", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
    
    detection.release_camera(cap)
    print("Final goal:", final_goal)
    print("Set points:", goal_set_points)

if __name__ == "__main__":
    main()
