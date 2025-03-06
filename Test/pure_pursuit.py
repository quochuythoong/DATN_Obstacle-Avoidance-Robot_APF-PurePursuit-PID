import math
import numpy as np
import apf_1st_implement as apf
from client_control import send_params
from utils import LookAHead_dist_RealLife, Wheels_dist, ConstVelocity

# Shared variables
flag_end_waypoint = False

def disable_pure_pursuit():
    global flag_end_waypoint
    flag_end_waypoint = True

def enable_pure_pursuit():
    global flag_end_waypoint
    flag_end_waypoint = False

def calculate_omega(AH, v, lt):
    omega = (2 * AH * v) / (lt ** 2)
    return omega

def calculate_wheel_velocities(omega, R, Ld):
    v1 = omega * (R + Ld)
    v2 = omega * (R - Ld)
    return v1, v2

def velocities_to_RPM(v1, v2):
    rpm1 = (v1 * 60) / (2 * math.pi * 0.0215) # radius of wheels = 0.0215 meter
    rpm2 = (v2 * 60) / (2 * math.pi * 0.0215)
    PWM1 = (rpm1 / 250) * 255  # 250 max RPM of motor, 255 max PWM of ESP
    PWM2 = (rpm2 / 250) * 255
    return PWM1, PWM2

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

    signed_distance = signed_distance * 0.0024245 # Convert pixel to meter in real life

    return projection_point.tolist(), signed_distance

def find_closest_point(current_position, waypoints, look_ahead_distance, error_tolerance=1.0):
    # Check if current_position is valid (has two coordinates)
    if not current_position or len(current_position) != 2:
        return None

    # # Convert waypoints into a NumPy array safely
    # try:
    #     waypoints = np.array(waypoints, dtype=np.float64)  # Convert safely
    # except ValueError:
    #     print("Error: Waypoints contain inconsistent shapes or types.")
    #     return None

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

    return closest_point

def pure_pursuit_main(aruco_coordinates, obstacle_coordinates, goal_set_points, end_point_arrow):
    path, potential_value = apf.apf_path_planning(aruco_coordinates, goal_set_points, obstacle_coordinates)
    closest_point = find_closest_point(aruco_coordinates, path, LookAHead_dist_RealLife)
    
    # Calculate the wheel velocities
    if closest_point:
        latest_waypoint = closest_point
        projection, signed_distance = calculate_signed_AH_and_projection(aruco_coordinates, end_point_arrow, latest_waypoint)   
        omega = calculate_omega(signed_distance, ConstVelocity, LookAHead_dist_RealLife)
        R = ConstVelocity / omega if omega != 0 else float('inf')
        v1, v2 = calculate_wheel_velocities(omega, R, Wheels_dist)
        PWM1, PWM2 = velocities_to_RPM(v1, v2)
        # print("PWM Left Wheel:", PWM1)
        # print("PWM Right Wheel:", PWM2)

    # Approaches the final point, stop the robot
    if flag_end_waypoint:
        PWM1 = 0
        PWM2 = 0

    # Send PWM1, PWM2 to client
    # send_params(PWM1, PWM2)