import numpy as np
import random
from scipy.ndimage import gaussian_filter1d

# Constants
step_size = 5

def attractive_potential(q, goal, k_att):
    return 0.5 * k_att * np.linalg.norm(q - goal) ** 2

def repulsive_potential(q, obstacles, k_rep, d0):
    U_rep = 0
    rep_grad = np.zeros_like(q)  

    for obs in obstacles:
        d = np.linalg.norm(q - obs)
        if d <= d0:
            U_rep += 0.5 * k_rep * (1/d - 1/d0) ** 2

            # Compute repulsive gradient (force)
            grad_rep = k_rep * (1/d - 1/d0) * (1/d**2) * (q - obs) / d  

            # Add a small tangential force (perpendicular to gradient)
            tangent = np.array([-grad_rep[1], grad_rep[0]])  
            rep_grad += grad_rep + 5 * tangent  

    return U_rep, rep_grad

def total_potential(q, goal, obstacles, k_att, k_rep, d0):
    U_att = attractive_potential(q, goal, k_att)
    U_rep, _ = repulsive_potential(q, obstacles, k_rep, d0)  # Ignore gradient
    return U_att + U_rep

def gradient(q, goal, obstacles, k_att, k_rep, d0, epsilon=1e-3):
    grad = np.zeros_like(q)

    # Compute attractive gradient
    for i in range(len(q)):
        q_step = q.copy()
        q_step[i] += epsilon  # Small step in dimension i
        grad[i] = (total_potential(q_step, goal, obstacles, k_att, k_rep, d0) - total_potential(q, goal, obstacles, k_att, k_rep, d0)) / epsilon
    
    grad = -grad  # Move in the negative gradient direction

    # Compute repulsive gradient
    _, rep_grad = repulsive_potential(q, obstacles, k_rep, d0)

    # Combine attractive and repulsive gradients
    grad += rep_grad  

    # If stuck in a local minimum, apply a small random perturbation
    if np.linalg.norm(grad) < 1e-3:  
        small_value = 0.2  # Fine-tuned perturbation range, can be adjusted
        grad += np.array([random.uniform(-small_value, small_value), 
                          random.uniform(-small_value, small_value)])
    
    # Limit step size
    if np.linalg.norm(grad) > (1/step_size):
        grad = grad / np.linalg.norm(grad)

    return grad

def apf_path_planning(start, goal, obstacles, k_att=0.0001, k_rep=100000.0, d0=50.0, max_iters=5000):
    path = [start]
    q = np.array(start, dtype=np.float64).flatten()
    goal = np.array(goal, dtype=np.float64).flatten()
    obstacles = np.array(obstacles, dtype=np.float64) if obstacles else np.empty((0, 2))
    potential_values = []

    for _ in range(max_iters):
        # Ensure q is valid before calling gradient()
        if not isinstance(q, np.ndarray) or q.shape != (2,):
            print(f"Error: Invalid q value {q}. Expected 2D array.")
            return [], []

        grad = gradient(q, goal, obstacles, k_att, k_rep, d0)

        # Normalize gradient to prevent large jumps
        grad_norm = np.linalg.norm(grad)
        if grad_norm > 0:
            grad = (grad / grad_norm) * step_size

        # Update position
        q += step_size * grad  # Move based on gradient
        path.append(q.copy())
        potential_values.append(total_potential(q, goal, obstacles, k_att, k_rep, d0))
        
        # Check if goal is reached
        if np.linalg.norm(q - goal) < 0.1:  # Goal reached threshold
            break

    # Convert path to a clean (N, 2) NumPy array
    path = np.array(path, dtype=np.float64).reshape(-1, 2)

    # Ensure path has enough points before smoothing
    if path.shape[0] > 1:
        path[:, 0] = gaussian_filter1d(path[:, 0], sigma=2)
        path[:, 1] = gaussian_filter1d(path[:, 1], sigma=2)

    return path, potential_values
