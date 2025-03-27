import numpy as np

def calculate_signed_AH_and_projection(A, B, C):
    A = np.array(A)
    B = np.array(B)
    C = np.array(C)
    AB = B - A
    AC = C - A
    dot_product = np.dot(AB, AC)
    AB_length_squared = np.dot(AB, AB)

    # Calculate the projection scalar t
    t = dot_product / AB_length_squared
    # Calculate the projection point coordinates
    projection_point = A + t * AB
    # Calculate signed distance from A to the projection point
    AH = projection_point - A
    dot_product_AH = np.dot(AB, AH)
    AH_magnitude = np.linalg.norm(AH)
    
    if dot_product_AH > 0:
        signed_distance = AH_magnitude  # H is on the same side as B with respect to A
    else:
        signed_distance = -AH_magnitude  # H is on the opposite side of B with respect to A

    signed_distance = signed_distance * 0.00102 # Convert pixel to meter in real life

    return projection_point.tolist(), signed_distance
