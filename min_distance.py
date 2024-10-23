import numpy as np
from scipy.optimize import minimize

def forward_kinematics(lengths, angles):
    x, y = 0, 0
    total_angle = 0
    for i in range(len(lengths)):
        total_angle += angles[i]
        x += lengths[i] * np.cos(total_angle)
        y += lengths[i] * np.sin(total_angle)
    return np.array([x, y])

def objective_function(angles, lengths):
    end_position = forward_kinematics(lengths, angles)
    return np.linalg.norm(end_position)

def calculate_minimum_distance(lengths, limits):
    # Initial angle is the max length
    initial_angles = [(lim[0] + lim[1]) / 2 for lim in limits]
    bounds = limits
    result = minimize(objective_function, initial_angles, args=(lengths,), bounds=bounds)
    
    if result.success:
        min_distance = result.fun
        optimal_angles = result.x.tolist()
        return min_distance, optimal_angles
    else:
        raise ValueError("Optimization failed")