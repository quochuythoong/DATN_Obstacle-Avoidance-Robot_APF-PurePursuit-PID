import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

# Example contour points
contour = np.array([[1, 1], [12, 1], [1, 12], [12, 2], [1.5, 12.5], [1.2, 11.5], [11.8, 1.5]], dtype=np.float32)

# Reshape the contour for cv2.fitEllipse: (N,1,2)
contour_reshaped = contour.reshape((-1, 1, 2))

# Fit an ellipse to the contour points using OpenCV
ellipse = cv2.fitEllipse(contour_reshaped)
center, axes, angle = ellipse
print("Fitted ellipse parameters:")
print("Center:", center)
print("Axes:", axes)
print("Angle:", angle)

# Define an expansion factor (e.g., 2 means the ellipse will be twice as big)
expansion_factor = 2

def ellipse_boundary_points(center, axes, angle, expansion_factor=1.0, num_points=100):
    """
    Computes the boundary points of an ellipse.
    
    Parameters:
      - center: (cx, cy) from cv2.fitEllipse.
      - axes: (major_axis, minor_axis) from cv2.fitEllipse.
      - angle: rotation angle in degrees from cv2.fitEllipse.
      - expansion_factor: scale factor to enlarge the ellipse.
      - num_points: number of boundary points to compute.
      
    Returns:
      - points: a (num_points, 2) array of (x,y) coordinates along the ellipse boundary.
    """
    # The fitted ellipse's width and height are scaled by the expansion factor.
    a = (axes[0] * expansion_factor) / 2.0  # semi-major axis
    b = (axes[1] * expansion_factor) / 2.0  # semi-minor axis
    theta = np.deg2rad(angle)  # convert rotation angle to radians
    t = np.linspace(0, 2 * np.pi, num_points)
    x = a * np.cos(t)
    y = b * np.sin(t)
    # Rotation matrix to rotate by theta
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta),  np.cos(theta)]])
    # Apply rotation and translation
    points = np.dot(R, np.vstack((x, y))).T + center
    return points

# Get the ellipse boundary points
boundary_points = ellipse_boundary_points(center, axes, angle, expansion_factor=expansion_factor, num_points=100)
print("Boundary points (first 5):")
print(boundary_points[:5])

# Plotting using Matplotlib
fig, ax = plt.subplots()

# Plot the original contour points
ax.scatter(contour[:, 0], contour[:, 1], color='blue', label='Contour Points')

# Draw the expanded ellipse using an Ellipse patch
ellipse_patch = Ellipse(xy=center,
                          width=axes[0] * expansion_factor,
                          height=axes[1] * expansion_factor,
                          angle=angle,
                          edgecolor='green',
                          fc='None',
                          lw=2,
                          label='Fitted Ellipse')
ax.add_patch(ellipse_patch)

# Plot the boundary points along the ellipse (red dots)
ax.plot(boundary_points[:, 0], boundary_points[:, 1], 'r.', markersize=3, label='Ellipse Boundary Points')

ax.set_aspect('equal')
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_title("Fitted Ellipse with Boundary Points")
ax.legend()

plt.show()
