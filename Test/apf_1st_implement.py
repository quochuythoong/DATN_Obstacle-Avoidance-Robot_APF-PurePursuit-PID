import numpy as np
import random
from scipy.ndimage import gaussian_filter1d

###############################################################################
# USER-ADJUSTABLE CONSTANTS
###############################################################################
step_size         = 1.0    # How far the robot moves each iteration
epsilon           = 1e-3   # Small step used in gradient/prediction
# q               = current position 
# goal            = target position 
# obstacles       = list of obstacle positions
# k_att           = attractive potential constant 
# k_rep           = repulsive potential constant
# d0              = repulsive potential range

def attractive_potential(q, goal, k_att):
    return 0.5 * k_att * np.linalg.norm(q - goal) ** 2

def repulsive_potential(q, obstacles, k_rep, d0):
    U_rep = 0 # U_rep = Repulsive potential (The amplitude of a specific point in the potential field)
    rep_grad = np.zeros_like(q)  # rep_grad = Repulsive gradient (The predictive_grad of the force at a specific point)

    for obs in obstacles:
        d = np.linalg.norm(q - obs)
        if d <= d0:
            U_rep += 0.5 * k_rep * (1/d - 1/d0) ** 2

            # Compute repulsive gradient (force)
            grad_rep = k_rep * (1/d - 1/d0) * (1/d**2) * (q - obs) / d  

            # Add a small tangential force (perpendicular to gradient)
            tangent = np.array([-grad_rep[1], grad_rep[0]])  
            rep_grad += grad_rep + 5 * tangent # 5 is a scaling factor for the tangential force

    return U_rep, rep_grad

def total_potential(q, goal, obstacles, k_att, k_rep, d0):
    U_att = attractive_potential(q, goal, k_att)
    U_rep, _ = repulsive_potential(q, obstacles, k_rep, d0)  # Ignore gradient
    return U_att + U_rep

def interpolate_waypoints(waypoints, step_distance=1.0):
    interpolated_points = []
    previous_point = None
    n = len(waypoints)

    if n < 2:
        return waypoints
    
    # Interpolate between consecutive points
    for i in range(n - 1):
        start = np.array(waypoints[i])
        end = np.array(waypoints[i + 1])
        distance = np.linalg.norm(end - start)
        if distance == 0:
            continue
        direction = (end - start) / distance
        num_steps = int(distance // step_distance) + 1

        for step in range(num_steps):
            interpolated_point = start + step * step_distance * direction
            rounded_point = (int(round(interpolated_point[0])), int(round(interpolated_point[1])))
            if rounded_point != previous_point:
                interpolated_points.append(rounded_point)
                previous_point = rounded_point

    return interpolated_points

# --- Basic APF ---
def basic_apf(q, goal, obstacles, k_att, k_rep, d0, epsilon, step_size):
    gradient = [q.copy()]
    
    # Compute APF force at current position
    grad = np.zeros_like(q)
    base_pot = total_potential(q, goal, obstacles, k_att, k_rep, d0)
    
    for i in range(len(q)):
        q_step = q.copy()
        q_step[i] += epsilon
        pot_diff = total_potential(q_step, goal, obstacles, k_att, k_rep, d0) - base_pot
        grad[i] = pot_diff / epsilon
    
    grad = -grad  # Move in descending potential
    _, rep_grad = repulsive_potential(q, obstacles, k_rep, d0)
    grad += rep_grad  # Add repulsive influence
    
    # Normalize and apply step size
    grad_norm = np.linalg.norm(grad)
    if grad_norm > 0:
        grad = (grad / grad_norm) * step_size
    
    # Update position and store in predicted path
    q = q + grad
    gradient.append(q.copy())

    # If gradient is very small => add random perturbation
    # if np.linalg.norm(gradient) < 1e-3:
    #     small_value = 0.2
    #     noise = np.array([random.uniform(-small_value, small_value),
    #                       random.uniform(-small_value, small_value)])
    #     gradient += noise
    #     gradient /= (np.linalg.norm(gradient) + 1e-9)

    return gradient

def apf_path_planning(start, goal, obstacles, k_att=0.0001, k_rep=100000.0, d0=50.0, max_iters=5000):
    """
    APF path planning using predictive APF for smooth navigation.
    """
    global epsilon, step_size
    path = [start]
    q = np.array(start, dtype=np.float64).flatten()
    goal = np.array(goal, dtype=np.float64).flatten()
    obstacles = np.array(obstacles, dtype=np.float64) if obstacles else np.empty((0, 2))
    
    for _ in range(max_iters):
        predicted_path = basic_apf(q, goal, obstacles, k_att, k_rep, d0, epsilon, step_size)
        
        # Choose the best predicted point (here: last one for smoothing)
        q = predicted_path[-1]
        path.append(q.copy())
        
        # Check if goal is reached
        if np.linalg.norm(q - goal) < 0.1:
            break
    
    path = np.array(path, dtype=np.float64).reshape(-1, 2)
    
    # Smooth the path
    if path.shape[0] > 1:
        path[:, 0] = gaussian_filter1d(path[:, 0], sigma=2)
        path[:, 1] = gaussian_filter1d(path[:, 1], sigma=2)
    
    path = interpolate_waypoints(path, step_distance=1.0)

    return path

