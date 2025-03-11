import matplotlib.pyplot as plt
import numpy as np

# Given list of points
points = [(1, 2), (2, 2), (2.5, 1.5), (3, 1), (4, 2), (5, 2), (6, 3), (7, 2)]
start = (1, 2)
goal = (7, 2)

# Function to calculate the perpendicular distance from a point to a line
def perpendicular_distance(point, line_start, line_end):
    x0, y0 = point
    x1, y1 = line_start
    x2, y2 = line_end
    
    numerator = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
    denominator = np.sqrt((y2 - y1)**2 + (x2 - x1)**2)
    
    return numerator / denominator

# Calculate the perpendicular distances for all points
distances = [perpendicular_distance(point, start, goal) for point in points]

# Find the points that are farthest from the line
# We will keep the start and goal points and add the points with the maximum distance
predictive_path = [start]
for i in range(1, len(points) - 1):
    if distances[i] > distances[i - 1] and distances[i] > distances[i + 1]:
        predictive_path.append(points[i])
predictive_path.append(goal)

# Plotting the original path
x_original, y_original = zip(*points)
plt.plot(x_original, y_original, 'bo-', label='Original Path')

# Plotting the predictive path
x_predictive, y_predictive = zip(*predictive_path)
plt.plot(x_predictive, y_predictive, 'ro-', label='Predictive Path')

# Plotting the line connecting start and goal
plt.plot([start[0], goal[0]], [start[1], goal[1]], 'g--', label='Start-Goal Line')

# Adding labels and legend
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Original Path vs Predictive Path')
plt.legend()
plt.grid(True)
plt.show()

# Output the predictive path
print("Predictive Path:", predictive_path)