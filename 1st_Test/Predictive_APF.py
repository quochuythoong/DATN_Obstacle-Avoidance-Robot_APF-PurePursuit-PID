# ###############################################################################
# # LIBRARIES
# ###############################################################################
import numpy as np
import math

# ###############################################################################
# # PREDICTIVE APF
# ###############################################################################

############-------- Deviation step & Trend changes Approach --------############

def perpendicular_distance(point, line_start, line_end):
    x0, y0 = point
    x1, y1 = line_start
    x2, y2 = line_end
    
    numerator = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
    denominator = math.sqrt((y2 - y1)**2 + (x2 - x1)**2)
    return numerator / denominator

def trend(a, b):
    if ((b - a) > 1):
        return 1  # increasing
    elif ((a - b) > 1):
        return -1  # decreasing
    else:
        return 0  # equal

def predictive_path(original_path, deviation_threshold):
    original_path = np.array(original_path)
    n = original_path.shape[0]
    if n < 2:
        return original_path
    
    pred_path = [original_path[0]]
    global_goal = original_path[-1]
    start = original_path[0]
    goal = original_path[-1]
    
    distances = []
    for pt in original_path:
        d = perpendicular_distance(pt, start, goal)
        distances.append(d)
    
    trend_changes = []
    prev_trend = None
    
    for i in range(deviation_threshold, len(distances), deviation_threshold):
        window_start = i - deviation_threshold
        window_end = i
        window_distances = distances[window_start:window_end]
        
        avg_start = np.mean(window_distances[:deviation_threshold//2])
        avg_end = np.mean(window_distances[deviation_threshold//2:])
        
        current_trend = trend(avg_start, avg_end)
        
        if prev_trend is None:
            prev_trend = current_trend
        else:
            if current_trend != prev_trend:
                # Trend changed at waypoint i-1, append the point (not the index)
                trend_changes.append(original_path[i - 1])
                prev_trend = current_trend
    
    # Remove the first point from trend_changes if it is the same as the start
    if len(trend_changes) > 0:
        del trend_changes[0] 

    for point in trend_changes:
        pred_path.append(point)
    
    if not np.array_equal(pred_path[-1], global_goal):
        pred_path.append(global_goal)
    
    return np.array(pred_path)

################################################################################

############-------- Original Approach --------############
# '''
# Overview of Predictive APF (PAPF):
# - PAPF will choose a temporary_goal (a point from the basic APF that has the greatest deviation from the start-goal line)
# - Go to that temporary_goal & set it as a new_start (creating a new_start-goal line)
# - Loop it until reaching the original Goal
# - Returning a smoother and more straightforward path planning

# Skeleton of the PAPF:
# def path_planning(...):
#     return path

# def predictive_path(original_path)
#     original_path = path_planning() # Original path (for example): [(2.5,6) (3,5.7) 4,5.5  5,4 6,3 7,3.5 8,4 9,4.5 10,4.8 11,5 12,4.5 13,4.2]
#     start = original_path[1]
#     goal = original_path[-1]
#     find greatest_deviation from start-goal # start-goal is a straight line from start to goal
#     plan path from start[2.5,6] to greatest_deviation from start-goal # in this case the greatest_deviation[6,3]
#         set the latest greatest_deviattion as a temporary_goal, 
#     after reaching the 1st temporary_goal
#         set it as a new_start
#     find next_greatest_deviation from new_start-goal # in this case, the next greatest_deviation is [11,5]
#         set the latest greatest deviation as a temporary_goal
#     after reaching the 1st temporary_goal
#         set it as a new_start
#     loop it until it reach the set original Goal [13,4.2] 
# '''

# def distance_from_line(pt, line_start, line_end):
#     """
#     Returns the perpendicular distance of point 'pt' from the line
#     defined by 'line_start' -> 'line_end'.
#     """
#     line_vec = line_end - line_start
#     line_len = np.linalg.norm(line_vec) + 1e-9  # prevent division by zero
#     pt_vec = pt - line_start
#     # 2D cross product magnitude: |x1*y2 - y1*x2|
#     cross_val = abs(line_vec[0]*pt_vec[1] - line_vec[1]*pt_vec[0])
#     return cross_val / line_len

# def predictive_path(original_path, deviation_threshold):
#     """
#     Given an original path (list/array of (x,y) points), this function splits the path
#     into segments by detecting the point with the greatest perpendicular deviation from
#     the straight line connecting the current start to the global goal.
    
#     It then returns a simplified predictive path consisting of:
#       - The starting point (from the ArUco)
#       - Temporary goals corresponding to the greatest deviation points
#       - The final goal
    
#     Parameters:
#         original_path: array-like shape (N, 2) of points
#         deviation_threshold: minimum deviation (perpendicular distance) required to create a temporary goal
        
#     Returns:
#         predictive_path: np.array of points representing the smoothed (predictive) path
#     """
#     original_path = np.array(original_path)
#     n = original_path.shape[0]
#     if n < 2:
#         return original_path
    
#     # Initialize the predictive path with the first point (e.g., the ArUco center)
#     pred_path = [original_path[0]]
#     current_index = 0
#     global_goal = original_path[-1]
    
#     # Loop until we reach the final goal
#     while current_index < n - 1:
#         start_pt = original_path[current_index]
#         # Consider all points from current_index+1 to the end
#         segment = original_path[current_index+1:]
        
#         # Compute perpendicular distances of every point in the original path from the start-goal line (start_pt -> global_goal) for the segment (Calculate deviation from line)
#         distances = np.array([distance_from_line(p, start_pt, global_goal) for p in segment])
#         max_idx_local = np.argmax(distances) # Find the index of the point in the distances array that has the maximum perpendicular deviation from the straight-line path (start → goal)
#         max_distance = distances[max_idx_local] # Input the value of that index (found in the above line)
        
#         # If the maximum deviation is significant, choose that point as a temporary goal
#         if max_distance > deviation_threshold:
#             temp_goal = segment[max_idx_local]
#             pred_path.append(temp_goal)

#             # Update current_index to the index of the temporary goal in original_path
#             current_index = current_index + 1 + max_idx_local 
#         else:
#             # If no significant deviation is found, break out of the loop
#             break
    
#     # Ensure that the global goal is included
#     if not np.array_equal(pred_path[-1], global_goal):
#         pred_path.append(global_goal)
    
#     return np.array(pred_path)