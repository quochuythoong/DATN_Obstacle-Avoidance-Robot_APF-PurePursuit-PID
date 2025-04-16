import matplotlib.pyplot as plt

# Define vectors
vectors = {
    "Left (-3, 0)": [-3, 0],
    "Down (0, -3)": [0, -3],
    "Up (0, 3)": [0, 3],
}

origin = [0], [0]  # Origin for all vectors

# Plot
plt.figure(figsize=(6, 6))
for label, vec in vectors.items():
    plt.quiver(*origin, vec[0], vec[1], angles='xy', scale_units='xy', scale=1, label=label)

plt.xlim(-5, 5)
plt.ylim(-5, 5)
plt.grid(True)
plt.axhline(0, color='black', lw=1)
plt.axvline(0, color='black', lw=1)
plt.gca().set_aspect('equal')
plt.title('Vector Visualization')
plt.legend()
plt.show()
