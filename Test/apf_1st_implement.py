import numpy as np
import random
from scipy.ndimage import gaussian_filter1d

###############################################################################
# USER-ADJUSTABLE CONSTANTS
###############################################################################
step_size         = 5    # How far the robot moves each iteration
epsilon           = 1e-3   # Small step used in gradient/prediction
# q               = current position 
# goal            = target position 
# obstacles       = list of obstacle positions
# k_att           = attractive potential constant 
# k_rep           = repulsive potential constant
# d0              = repulsive potential range

###############################################################################
# ADD-ON FUNCTIONS
###############################################################################
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

###############################################################################
# CALCULATE POTENTIAL FIELD
###############################################################################
def attractive_potential(q, goal, k_att):
    return 0.5 * k_att * np.linalg.norm(q - goal) ** 2

def repulsive_potential(q, obstacles, k_rep, d0):
    U_rep = 0 # U_rep = Repulsive potential (The amplitude of a specific point in the potential field)
    rep_grad_left = 0
    rep_grad_right = 0

    for obs in obstacles:
        d = np.linalg.norm(q - obs)
        if d <= d0:
            U_rep += 0.5 * k_rep * (1/d - 1/d0) ** 2

            # Compute repulsive gradient (force)
            grad_rep = k_rep * (1/d - 1/d0) * (1/d**2) * (q - obs) / d  

            # Add a small tangential force (perpendicular to gradient)
            tangent_rep = np.array([-grad_rep[1], grad_rep[0]])  

            # '-' sign for turning left / '+' sign for turning right
            # Step ahead for a number of steps, calculate the APF at that point
            # If the distance is farther from Goal --> Avoid 
            # If the distance is closer from Goal  --> Go
            # If the APF is the same --> Randomly choose to go or avoid
            rep_grad_left -= grad_rep + 5 * tangent_rep # 5 is a scaling factor for the tangential force
            rep_grad_right += grad_rep + 5 * tangent_rep

    return U_rep, rep_grad_left, rep_grad_right

def total_potential(q, goal, obstacles, k_att, k_rep, d0):
    U_att = attractive_potential(q, goal, k_att)
    U_rep, _, _ = repulsive_potential(q, obstacles, k_rep, d0)  # Ignore gradient
    return U_att + U_rep

###############################################################################
# BASIC ARTIFICIAL POTENTIAL FIELD (APF) GRADIENT & LOWER-FIELD ORIENTATION SLIP CHOICES
###############################################################################
def basic_apf(q, goal, obstacles, k_att, k_rep, d0, epsilon, step_size):
    gradient = 0
    _, rep_grad_left, rep_grad_right = repulsive_potential(q, obstacles, k_rep, d0) # rep_grad = Repulsive gradient (The predictive_grad of the force at a specific point)
    
    # Compute APF force at current position
    grad = np.zeros_like(q)
    base_pot = total_potential(q, goal, obstacles, k_att, k_rep, d0)
    
    for i in range(len(q)):
        q_step = q.copy()
        q_step[i] += epsilon
        pot_diff = total_potential(q_step, goal, obstacles, k_att, k_rep, d0) - base_pot
        grad[i] = pot_diff / epsilon
    
    grad = -grad  # Move in descending potential
    
    grad_left = grad + rep_grad_left  # Add repulsive influence
    grad_right = grad + rep_grad_right  # Add repulsive influence

    # Normalize and apply step size
    grad_norm_left = np.linalg.norm(grad_left)
    grad_norm_right = np.linalg.norm(grad_right)
    if grad_norm_left > 0:
        grad_left_1 = (grad_left / grad_norm_left) * step_size
    if grad_norm_right > 0:
        grad_right_1 = (grad_right / grad_norm_right) * step_size

    # Update position and store in predicted path
    q_left = q + grad_left_1
    q_right = q + grad_right_1

    d_q_left = np.linalg.norm(q_left - goal)
    d_q_right = np.linalg.norm(q_right - goal)

    if d_q_left < d_q_right:
        gradient = grad_left
    elif d_q_right < d_q_left:
        gradient = grad_right
    else:
        gradient = random.choice([grad_left, grad_right])

    # Normalize and apply step size
    grad_norm = np.linalg.norm(gradient)
    if grad_norm > 0:
        gradient = (gradient / grad_norm) * step_size

    return gradient

###############################################################################
# APF PATH PLANNING
###############################################################################
def apf_path_planning(start, goal, obstacles, k_att=0.0001, k_rep=100000.0, d0=90.0, max_iters=5000):
    global epsilon, step_size
    path = [start]
    q = np.array(start, dtype=np.float64).flatten()
    goal = np.array(goal, dtype=np.float64).flatten()
    obstacles = np.array(obstacles, dtype=np.float64) if obstacles else np.empty((0, 2))
    
    # potential_values = []
    
    for _ in range(max_iters):
        basic_apf_calculated = basic_apf(q, goal, obstacles, k_att, k_rep, d0, epsilon, step_size)
        q += step_size * basic_apf_calculated  # Move based on gradient
        path.append(q.copy())
        # potential_values.append(total_potential(q, goal, obstacles, k_att, k_rep, d0))
        
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
