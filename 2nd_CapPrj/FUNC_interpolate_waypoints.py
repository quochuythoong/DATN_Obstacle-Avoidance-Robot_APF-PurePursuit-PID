import numpy as np

def interpolate_waypoints(waypoints, step_distance=1.0):
    interpolated_points = []
    previous_point = None

    for i in range(len(waypoints) - 1):
        start = np.array(waypoints[i])
        end = np.array(waypoints[i + 1])
        distance = np.linalg.norm(end - start)
        direction = (end - start) / distance
        num_steps = int(distance // step_distance) + 1

        for step in range(num_steps):
            interpolated_point = start + step * step_distance * direction
            rounded_point = (int(round(interpolated_point[0])), int(round(interpolated_point[1])))

            if rounded_point != previous_point:
                interpolated_points.append(rounded_point)
                previous_point = rounded_point

    last_point = (int(round(waypoints[-1][0])), int(round(waypoints[-1][1])))
    if last_point != previous_point:
        interpolated_points.append(last_point)

    return interpolated_points
