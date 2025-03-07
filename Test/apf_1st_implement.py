import numpy as np
import random
from scipy.ndimage import gaussian_filter1d

# Constants
step_size = 1
epsilon = 1e-3
prediction_horizon = 10
# q = current position, goal = target position, obstacles = list of obstacle positions
# k_att = attractive potential constant, k_rep = repulsive potential constant, d0 = repulsive potential range

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

''' --------------------------------- BASIC APF GRADIENT CALCULATION ---------------------------------
def gradient(q, goal, obstacles, k_att, k_rep, d0, epsilon):
    grad = np.zeros_like(q)

    # Compute attractive gradient
    for i in range(len(q)):
        q_step = q.copy()
        q_step[i] += epsilon  # Small step in dimension i
        grad[i] = (total_potential(q_step, goal, obstacles, k_att, k_rep, d0) - total_potential(q, goal, obstacles, k_att, k_rep, d0)) / epsilon
    
    grad = -grad  # Move in the negative gradient predictive_grad

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
--------------------------------- BASIC APF GRADIENT CALCULATION ---------------------------------'''

def distance_from_line(pt, line_start, line_end):
    """
    Returns the perpendicular distance of point `pt` from the line
    defined by line_start -> line_end.

    Detect deviation from a straight line (q -> goal) by measuring
    the perpendicular distance of a point from the line.
    """
    # Parametric or vector form: line(t) = line_start + t*(line_end - line_start)
    # Distance formula is cross((pt - start), (end - start)) / |end - start|
    line_vec = line_end - line_start
    line_len = np.linalg.norm(line_vec) + 1e-9  # avoid zero-div
    pt_vec   = pt - line_start
    
    # Cross product in 2D => (x1*y2 - y1*x2) in absolute value
    cross_val = abs(line_vec[0]*pt_vec[1] - line_vec[1]*pt_vec[0])
    return cross_val / line_len

def predictive_gradient_greatest_deviation(q, goal, obstacles,
                                           k_att, k_rep, d0,
                                           epsilon, prediction_horizon):
    """
    Predictive Gradient that picks as 'temp_goal' the point with the greatest
    deviation from the straight line (q -> goal).
    
    Steps:
    1) Compute a basic APF gradient at q.
    2) Predict a short path forward in small steps (prediction_horizon).
    3) Measure the perpendicular distance of each predicted point
       from the line q->goal, pick the one with the largest distance.
    4) The predictive_grad from q to that 'max deviation point' is returned
       as the predictive gradient.
    """
    # --- 1) Basic APF gradient at q (finite differences + repulsive) ---
    grad = np.zeros_like(q)
    base_pot = total_potential(q, goal, obstacles, k_att, k_rep, d0)
    for i in range(len(q)):
        q_step = q.copy()
        q_step[i] += epsilon
        pot_diff = total_potential(q_step, goal, obstacles, k_att, k_rep, d0) - base_pot
        grad[i] = pot_diff / epsilon
    grad = -grad  # negative => descend potential
    
    # Add repulsive gradient
    _, rep_grad = repulsive_potential(q, obstacles, k_rep, d0)
    grad += rep_grad

    # --- 2) Predict short path in small steps along 'grad' ---
    predicted_points = []
    q_sim = q.copy()
    for _ in range(prediction_horizon):
        predicted_points.append(q_sim.copy())
        q_sim += epsilon * grad  # small step along gradient

    if len(predicted_points) == 0:
        # fallback: just use grad
        return grad / (np.linalg.norm(grad) + 1e-9)

    # --- 3) Pick the predicted point with greatest deviation from line (q->goal) ---
    max_dist = -1.0
    max_idx = 0
    for i, p in enumerate(predicted_points):
        dist = distance_from_line(p, q, goal)
        if dist > max_dist:
            max_dist = dist
            max_idx = i

    temp_goal = predicted_points[max_idx]

    # --- 4) Return predictive_grad from q to that 'temp_goal' ---
    predictive_grad = temp_goal - q
    norm_dir = np.linalg.norm(predictive_grad)
    if norm_dir > 1e-9:
        predictive_grad /= norm_dir
    else:
        # fallback: use original grad
        g_norm = np.linalg.norm(grad)
        predictive_grad = grad / (g_norm + 1e-9)

    # If predictive_grad is very small => add random perturbation
    if np.linalg.norm(predictive_grad) < 1e-3:
        small_value = 0.2
        noise = np.array([random.uniform(-small_value, small_value),
                          random.uniform(-small_value, small_value)])
        predictive_grad += noise
        predictive_grad /= (np.linalg.norm(predictive_grad) + 1e-9)

    return predictive_grad

def apf_path_planning(start, goal, obstacles, k_att=0.0001, k_rep=100000.0, d0=50.0, max_iters=5000):
    global epsilon, step_size, prediction_horizon

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

        grad =  predictive_gradient_greatest_deviation(q, goal, obstacles, k_att, k_rep, d0, epsilon, prediction_horizon)

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
