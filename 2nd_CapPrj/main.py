import cv2
import numpy as np
import openCV
import math
import time
from client_control import send_params, send_PID, ena_PID
from utils import frame_height, frame_width, start_button_pos, reset_button_pos, button_width, button_height, ConstVelocity, Wheels_dist, LookAHead_dist
from pure_pursuit import calculate_omega, calculate_wheel_velocities, velocities_to_RPM, calculate_adaptive_lookahead
from FUNC_mouse_callback import mouse_callback
from FUNC_interpolate_waypoints import interpolate_waypoints
from FUNC_find_closest_point import find_closest_point
from FUNC_calculate_signed_AH_and_projection import calculate_signed_AH_and_projection
from FUNC_draw_buttons import draw_buttons

# Shared variables
clicked_points = []
center_coordinate = ()
end_point_arrow = ()
start_pressed = [False]
interpolated_waypoints = []
flag_clicked_point_added = False
flag_end_waypoint = False
latest_waypoint = ()
distance_current = 0
Adaptive_lookahead = 0
LookAHead_dist_current = LookAHead_dist
corners_save = None
angle_save = None
max_angle_different = 90
omega = 0
w1 = 0
w2 = 0
closest_point_save = ()
# Setup camera and window
cap = openCV.initialize_camera()
openCV.initialize_window(
    "2D ArUco Marker Detection and Vector",
    mouse_callback,
    (clicked_points, reset_button_pos, start_button_pos, button_width, button_height,
      frame_height, start_pressed, interpolated_waypoints,latest_waypoint)
)

# ArUco setup
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
parameters = cv2.aruco.DetectorParameters()  

# PID Parameters
kp = 12
ki = 4
kd = 0.1
send_PID(kp, ki, kd)
ena_PID(0) # Disable PID on motors initially

#__main__
while True:
    # Draw buttons
    result = openCV.process_frame(cap)
    if result is None:
        break
    frame, gray = result
    draw_buttons(frame, start_button_pos, reset_button_pos, button_width, button_height)
    # Detect ArUco markers
    corners, ids = openCV.detect_aruco_markers(gray, aruco_dict, parameters)

    # Draw center and orientation
    if corners:
        center_coordinate, end_point_arrow, angle = openCV.draw_center_and_orientation(frame, corners, frame_height, frame_width)
        # if (angle - angle_save) > max_angle_different:
        #     center_coordinate, end_point_arrow, angle = openCV.draw_center_and_orientation(frame, corners_save, frame_height, frame_width)
        # else:
        #     corners_save = corners
        # angle_save = angle
    
    # Draw clicked points
    if clicked_points:
        openCV.draw_clicked_points(frame, clicked_points, frame_height)

    if len(clicked_points) > 1 and start_pressed[0] and ids is not None:
        if (len(interpolated_waypoints) < 1) and (len(clicked_points) >= 1):
            if flag_clicked_point_added == False:
                clicked_points.insert(0,center_coordinate)
                flag_clicked_point_added = True
                interpolated_waypoints =interpolate_waypoints(clicked_points)

        if interpolated_waypoints:
            closest_point, interpolated_waypoints = find_closest_point(center_coordinate, interpolated_waypoints, LookAHead_dist_current)
            if closest_point:
                latest_waypoint = closest_point
                projection, signed_distance = calculate_signed_AH_and_projection(center_coordinate, end_point_arrow, latest_waypoint)
                # Calculate omega and wheel velocities
                omega = calculate_omega(signed_distance, ConstVelocity, LookAHead_dist_current)
                R = ConstVelocity / omega if omega != 0 else float('inf')
                w1, w2 = calculate_wheel_velocities(omega, R, Wheels_dist)
                # w1, w2 = velocities_to_RPM(v1, v2)
                #print(f"Left: {w1}, Right: {w2}")

        if len(interpolated_waypoints) <= 10:
            distance_current = math.sqrt((latest_waypoint[0] - center_coordinate[0])**2 + (latest_waypoint[1] - center_coordinate[1])**2)
            if distance_current <= 15:
                flag_end_waypoint = True
            else:
                projection, signed_distance = calculate_signed_AH_and_projection(center_coordinate, end_point_arrow, latest_waypoint)
                # print(f"Projection: {projection}, Signed Distance: {signed_distance}")
                # Calculate omega and wheel velocities
                omega = calculate_omega(signed_distance, ConstVelocity, LookAHead_dist_current)
                R = ConstVelocity / omega if omega != 0 else float('inf')
                w1, w2 = calculate_wheel_velocities(omega, R, Wheels_dist)
                # w1, w2 = velocities_to_RPM(v1, v2)
                # print("PWM Left Wheel:", w1)
                # print("PWM Right Wheel:", w2)
        
        frame = openCV.draw_detected_markers(frame, corners, ids) 

        ena_PID(1) # Enable PID on motors

    cv2.line(frame, 
                 (int(center_coordinate[0]), int(-(center_coordinate[1]-frame_height))), 
                 (int(latest_waypoint[0]), int(-(latest_waypoint[1]-frame_height))), 
                 (0, 255, 255), 2)
    # Use adaptive look ahead for next loop
    Adaptive_lookahead = calculate_adaptive_lookahead(w1, w2, omega)
    LookAHead_dist_current = Adaptive_lookahead * 0.00102

    if start_pressed[0] == False:
        #print("Add point")
        flag_clicked_point_added = False
        flag_end_waypoint = False
        ena_PID(0)
        w1 = 0
        w2 = 0
    # Approaches the final point, stop the robot
    if flag_end_waypoint:
        ena_PID(0)
        w1 = 0
        w2 = 0
        clicked_points.clear()
        interpolated_waypoints.clear()
    # Send w1, w2 to client
    send_params(w1, w2)
    # send_params(w1, w2, 15, 4, 1.5)

    # Display the frame
    cv2.imshow("2D ArUco Marker Detection and Vector", frame)
    
    # Press "ESC" to close the window
    if cv2.waitKey(1) & 0xFF == 27:
        break

openCV.release_camera(cap)
ena_PID(0)
send_params(0, 0)
# send_params(w1, w2, 15, 4, 1.5)
