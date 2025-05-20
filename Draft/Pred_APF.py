import matplotlib.pyplot as plt
import math
# Example waypoints as a list of (x, y) tuples
waypoints = [
    (1, 2),
    (2, 1.5),
    (2, 3),
    
    (2, 5),
    (2, 6),
    (2, 7),
    (2, 8),
    (2, 9),
    (2, 10),
    (2,15),
    (2.5, 15),
    (3, 15),
    (3.5, 15),
    (4, 15),
    (4, 16),
    (4, 17),
    (4, 18),
    (4, 19),
    (20,40)
]

start = waypoints[0]
goal = waypoints[-1]

def perpendicular_distance(point, line_start, line_end):
    x0, y0 = point
    x1, y1 = line_start
    x2, y2 = line_end
    
    numerator = abs((y2 - y1)*x0 - (x2 - x1)*y0 + x2*y1 - y2*x1)
    denominator = math.sqrt((y2 - y1)**2 + (x2 - x1)**2)
    return numerator / denominator

# Calculate distances
distances = []
for pt in waypoints:
    d = perpendicular_distance(pt, start, goal)
    distances.append(d)

# Detect trend changes (increasing <-> decreasing)
trend_changes = []

# Determine initial trend (None if equal)
def trend(a, b):
    if b > a:
        return 1  # increasing
    elif b < a:
        return -1  # decreasing
    else:
        return 0  # equal

prev_trend = None
for i in range(1, len(distances)):
    current_trend = trend(distances[i-1], distances[i])
    if current_trend == 0:
        # equal, no change, continue
        continue
    if prev_trend is None:
        prev_trend = current_trend
    else:
        if current_trend != prev_trend:
            # Trend changed at waypoint i
            trend_changes.append(i - 1)
            prev_trend = current_trend

# Print trend change points
print("Trend change waypoints (index, coordinates):")
for idx in trend_changes:
    print(f"Waypoint {idx} at {waypoints[idx]}")

# Plotting
plt.figure(figsize=(8, 6))
x_coords, y_coords = zip(*waypoints)
plt.scatter(x_coords, y_coords, color='red', label='Waypoints')
plt.plot(x_coords, y_coords, linestyle='-', color='blue', label='Path')
plt.plot([start[0], goal[0]], [start[1], goal[1]], linestyle='--', color='green', label='Start to Goal')

# Draw perpendicular lines from each point to the line
for pt in waypoints:
    x0, y0 = pt
    x1, y1 = start
    x2, y2 = goal
    
    dx = x2 - x1
    dy = y2 - y1
    
    t = ((x0 - x1)*dx + (y0 - y1)*dy) / (dx*dx + dy*dy)
    
    x_proj = x1 + t*dx
    y_proj = y1 + t*dy
    
    plt.plot([x0, x_proj], [y0, y_proj], linestyle=':', color='gray')

# Mark trend change points on the plot
for idx in trend_changes:
    x, y = waypoints[idx]
    plt.scatter(x, y, color='orange', s=150, edgecolors='black', label='Trend Change' if idx == trend_changes[0] else "")
    plt.text(x, y, f"  {idx}", fontsize=12, verticalalignment='bottom')

plt.title('Waypoints, Path, Start to Goal Line and Trend Changes in Distance')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.grid(True)
plt.show()