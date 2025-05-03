import numpy as np
center_coordinate =[636 ,321]
latest_waypoint =[3,3]
center = np.array(center_coordinate)

# Extract top-left and top-right corners


# Calculate midpoint
midpoint_x = (top_left[0] + top_right[0]) / 2
midpoint_y = (top_left[1] + top_right[1]) / 2
midpoint = [midpoint_x, midpoint_y]

head = np.array(midpoint)
first_location = np.array(latest_waypoint)

# Calculate vectors
robot_direction = head - center
location_direction = first_location - center
# print(f"robot_direction: {robot_direction}")
# print(f"location_direction: {location_direction}")
# Check for zero vectors
if np.all(robot_direction == 0) or np.all(location_direction == 0):
    print("Error: Invalid direction vectors (zero vector detected).")
    w1, w2 = 0, 0
    flag_initialize_direction = True
else:
    # Cross product
    cross_product = robot_direction[0] * location_direction[1] - robot_direction[1] * location_direction[0]

    # Angle calculation with numerical stability
    cos_theta = np.dot(robot_direction, location_direction) / (
        np.linalg.norm(robot_direction) * np.linalg.norm(location_direction)
    )
    cos_theta = np.clip(cos_theta, -1.0, 1.0)  # Prevent numerical errors
    angle_deg = np.degrees(np.arccos(cos_theta))
    # print(f"angle:{angle_deg}")
    # Alignment check
    if angle_deg <= 15:
        flag_initialize_direction = True
        w1, w2 = 0, 0  # Stop turning
    else:
        # Handle cross product = 0 (parallel or anti-parallel)
        if abs(cross_product) < 1e-10:  # Numerical threshold for zero
            if angle_deg > 90:  # 180° case: choose a default turn (e.g., clockwise)
                w1, w2 = 0, 2  # Clockwise
            else:
                w1, w2 = 0, 0  # Already aligned (should be caught by angle_deg <= 15)
        else:
            # Normal turn logic
            if cross_product > 0:  # Counterclockwise
                w1, w2 = 0, 2  # Right wheel on, left wheel off
            else:  # Clockwise
                w1, w2 = 2, 0  # Left wheel on, right wheel off

            print(f"Initualizing direction angle: {angle_deg} cross: {cross_product} ")