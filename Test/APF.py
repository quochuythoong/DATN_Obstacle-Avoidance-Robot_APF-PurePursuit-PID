# APF.py
import numpy as np
import cv2

def compute_APF(goal, obstacles, frame_size):
    mesh = np.zeros((frame_size[1], frame_size[0], 3), dtype=np.uint8)
    if goal is None:
        return mesh
    for y in range(frame_size[1]):
        for x in range(frame_size[0]):
            repulsion = sum(1000 / ((x - ox) ** 2 + (y - oy) ** 2 + 1) for ox, oy, _, _ in obstacles) if obstacles else 0
            attraction = ((x - goal[0]) ** 2 + (y - goal[1]) ** 2) * 0.0001
            intensity = min(255, int(repulsion + attraction))
            mesh[y, x] = (intensity, intensity, 255 - intensity)
    return mesh

# Nothing yet, do not use this file for now