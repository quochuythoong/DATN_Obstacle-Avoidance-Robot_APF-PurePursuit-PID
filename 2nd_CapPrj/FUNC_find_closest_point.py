import numpy as np

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
    # print("Waypoints:", len(waypoints))

    if closest_index >= 0:
        tempList=  tempList[closest_index:]

    # print("Waypoints:", len(waypoints))

    return closest_point, tempList