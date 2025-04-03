###############################################################################
# LIBRARIES
###############################################################################
import math
import numpy as np
import math
import cv2
import aruco_obstacle_detection as detection
from client_control import send_params, ena_PID
from utils import Wheels_dist, ConstVelocity, frame_height, frame_width, k1, k2, min_ld, wheel_scale, real_ld_scale, max_ld

###############################################################################
# GLOBAL VARIABLES
###############################################################################
global center_coordinate, end_point_arrow
flag_end_waypoint = False
distance_current = 0
latest_waypoint = ()
aruco_path = None
Adaptive_LookAHead_pixels = min_ld  # Adaptive lookahead distance in pixels
LookAHead_dist_current = min_ld * real_ld_scale    # Real-life lookahead distance in meters
omega = 0
w1 = 0
w2 = 0

###############################################################################
# ENABLE / DISABLE PURE PURSUIT
###############################################################################
def disable_pure_pursuit():
    global flag_end_waypoint
    flag_end_waypoint = True

def enable_pure_pursuit():
    global flag_end_waypoint
    flag_end_waypoint = False

###############################################################################
# PURE PURSUIT CALCULATION
###############################################################################
def calculate_omega(AH, v, lt):
    omega = (2 * AH * v) / (lt ** 2)
    return omega

def calculate_wheel_velocities(omega, R, Ld):
    if abs(omega) < 1e-6:
        v1 = ConstVelocity
        v2 = ConstVelocity
    else:
        v1 = omega * (R + Ld)
        v2 = omega * (R - Ld)

    w1 = v1 / 0.0215 # rad/s
    w2 = v2 / 0.0215

    # Limit negative values
    if w1 < 0:
        w1 = 0
    elif w2 < 0:
        w2 = 0

    # Limit wheel velocities
    if w1 > 10:
        w1 = 10
    if w2 > 10:
        w2 = 10

    print(f"Wheel 1: {w1}, Wheel 2: {w2}")
    return w1, w2

def calculate_signed_AH_and_projection(A, B, C):
    A = np.array(A)
    B = np.array(B)
    C = np.array(C)
    AB = B - A
    AC = C - A
    dot_product = np.dot(AB, AC)
    AB_length_squared = np.dot(AB, AB)

    # Calculate the projection scalar t
    t = dot_product / AB_length_squared
    # Calculate the projection point coordinates
    projection_point = A + t * AB
    # Calculate signed distance from A to the projection point
    AH = projection_point - A
    dot_product_AH = np.dot(AB, AH)
    AH_magnitude = np.linalg.norm(AH)
    
    if dot_product_AH > 0:
        signed_distance = AH_magnitude  # H is on the same side as B with respect to A
    else:
        signed_distance = -AH_magnitude  # H is on the opposite side of B with respect to A

    signed_distance = signed_distance * real_ld_scale # Convert pixel to meter in real life

    return projection_point.tolist(), signed_distance

def find_closest_point(current_position, waypoints, look_ahead_distance, error_tolerance=1.0):
    # Check if current_position is valid (has two coordinates)
    if not current_position or len(current_position) != 2:
        return None

    closest_point = None
    min_distance_diff = float('inf')
    closest_index = -1
    tempList = waypoints.copy()

    for i, point in enumerate(waypoints):
        if i < (look_ahead_distance + 20):
            distance = np.linalg.norm(np.array(current_position) - np.array(point))
            distance_diff = abs(distance - look_ahead_distance)
            
            if distance_diff < min_distance_diff and distance_diff <= error_tolerance:
                min_distance_diff = distance_diff
                closest_point = point
                closest_index = i

    if closest_index >= 0:
        tempList = tempList[closest_index:]

    return closest_point, tempList

###############################################################################
# ADAPTIVE LOOKAHEAD
###############################################################################
def calculate_adaptive_lookahead(w1, w2, omega):
    global k1, k2, min_ld

    # Robot velocity related to 2 wheels velocity
    v_robot = wheel_scale * (w1 + w2)

    # Tuned lookahead distance
    if omega < 0: # Negative omega --> turn right
        tuned_ld = (k1 * v_robot) - (k2 * omega) 
    else: # Positive omega --> turn left
        tuned_ld = (k1 * v_robot) + (k2 * omega)

    ld = max(tuned_ld, min_ld)  # Ensure lookahead is not below min_ld

    # Limit lookahead distance
    if ld > max_ld:
        ld = max_ld
        
    return ld

###############################################################################
# PURE PURSUIT MAIN EXECUTION
###############################################################################
def pure_pursuit_main(corners, global_path, frame):
    global flag_end_waypoint, distance_current, w1, w2, center_coordinate, latest_waypoint, end_point_arrow, omega, LookAHead_dist_current, Adaptive_LookAHead_pixels, aruco_path

    # Draw center and orientation of the robot
    if corners:
        center_coordinate, end_point_arrow, angle = detection.calculate_center_and_orientation(corners, frame_height)
        detection.draw_center_and_orientation_display(frame, center_coordinate, angle, end_point_arrow, Adaptive_LookAHead_pixels, frame_width, frame_height)
        aruco_path = detection.aruco_path_plot(frame, center_coordinate, flag_end_waypoint)

    # Pure Pursuit approaching the last waypoints
    if len(global_path) <= 10:
        distance_current = math.sqrt((latest_waypoint[0] - center_coordinate[0])**2 + (latest_waypoint[1] - center_coordinate[1])**2)
        if distance_current <= 30: # Last detected waypoints
            flag_end_waypoint = True
        else:
            projection, signed_distance = calculate_signed_AH_and_projection(center_coordinate, end_point_arrow, latest_waypoint)

            # Calculate omega and wheel velocities
            omega = calculate_omega(signed_distance, ConstVelocity, LookAHead_dist_current)
            R = ConstVelocity / omega if omega != 0 else float('inf')
            w1, w2 = calculate_wheel_velocities(omega, R, Wheels_dist)

    # Continuous Pure Pursuit
    if global_path:
        closest_point, global_path = find_closest_point(center_coordinate, global_path, Adaptive_LookAHead_pixels)
        if closest_point:
            print(f"closest_point: {closest_point}")
            latest_waypoint = closest_point
            cv2.line(frame, 
                (int(center_coordinate[0]), int(-(center_coordinate[1]-frame_height))), 
                (int(latest_waypoint[0]), int(-(latest_waypoint[1]-frame_height))), 
                (0, 255, 255), 2)

            projection, signed_distance = calculate_signed_AH_and_projection(center_coordinate, end_point_arrow, latest_waypoint)

            # Calculate omega and wheel velocities
            omega = calculate_omega(signed_distance, ConstVelocity, LookAHead_dist_current)
            R = ConstVelocity / omega if omega != 0 else float('inf')
            w1, w2 = calculate_wheel_velocities(omega, R, Wheels_dist)

    # Use adaptive look ahead for next loop
    Adaptive_LookAHead_pixels = calculate_adaptive_lookahead(w1, w2, omega)
    LookAHead_dist_current = Adaptive_LookAHead_pixels * real_ld_scale

    # Approaches the final point, stop the robot - else keep moving
    if flag_end_waypoint:
        ena_PID(0)  # Disable PID
        w1 = 0
        w2 = 0

    # Send w1, w2 to client
    send_params(w1, w2)

    return global_path, aruco_path