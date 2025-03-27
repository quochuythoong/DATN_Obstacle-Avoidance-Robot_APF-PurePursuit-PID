###############################################################################
# LIBRARIES
###############################################################################
import numpy as np

###############################################################################
# FRAME DIMENSIONS
###############################################################################
frame_height = 640     # Frame height
frame_width  = 480     # Frame width

###############################################################################
# PURE PURSUIT PARAMETERS
###############################################################################

# Robot parameters
ConstVelocity = 0.055   # Constant velocity in m/s (0.1505m/s = 7rad/s * )
Wheels_dist   = 0.05    # Distance between wheels in meters
wheel_scale   = 0.01075 # R / 2 = 0.0215 / 2 = 0.01075

# Adaptive Look-ahead parameters 
k1            = 80 / ConstVelocity                     # Gain for the linear velocity (v)
k2            = - 60 / (2 * ConstVelocity / 0.0215)    # Gain for the angular velocity (w)
min_ld        = 45                                     # Minimum look-ahead distance (pixels)

###############################################################################
# PID PARAMETERS
###############################################################################
kp            = 5       # Proportional 
ki            = 3       # Integral
kd            = 0       # Derivative

###############################################################################
# ADD-ON FUNCTIONS (Interpolate)
###############################################################################
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

    return interpolated_points

def interpolate_waypoints_contour(waypoints, step_distance=1.0):
    interpolated_points_contour = []
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
                interpolated_points_contour.append(rounded_point)
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
                interpolated_points_contour.append(rounded_point)
                previous_point = rounded_point

    return interpolated_points_contour