from my_math import *
from min_distance import calculate_minimum_distance
import numpy as np

LEFT = False
RIGHT = True

def ik(x, y, lengths, constraints = []):      # Result is absolute orientation in rad  # Constraint is a list of tuples    
    def ik_2links(x, y, l1, l2, direction):
        cos_theta2 = constrain_sin_cos((x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2))
        sin_theta2 = sqrt(1 - cos_theta2**2) if direction else -sqrt(1 - cos_theta2**2) # Choose the desired direction
        theta2 = atan2(sin_theta2, cos_theta2)
        
        k1 = l1 + l2 * cos_theta2
        k2 = l2 * sin_theta2
        theta1 = atan2(y, x) - atan2(k2, k1)

        # print("2 way ik,", theta1, theta2)
        
        return [theta1, theta2]

    def ik_3links(x, y, L1, L2, L3):
        # Calculate the position of the wrist (ignoring the third link)
        wrist_x = x - L3 * np.cos(np.arctan2(y, x))
        wrist_y = y - L3 * np.sin(np.arctan2(y, x))

        # Use the cosine rule to find the angles
        cos_theta2 = (wrist_x**2 + wrist_y**2 - L1**2 - L2**2) / (2 * L1 * L2)
        theta2 = np.arccos(np.clip(cos_theta2, -1.0, 1.0))  # Clipping to avoid numerical errors

        theta1 = np.arctan2(wrist_y, wrist_x) - np.arctan2(L2 * np.sin(theta2), L1 + L2 * np.cos(theta2))

        theta3 = np.arctan2(y, x) - theta1 - theta2

        # Convert angles from radians to degrees
        theta1 = np.degrees(theta1)
        theta2 = np.degrees(theta2)
        theta3 = np.degrees(theta3)

        return theta1, theta2, theta3

    def get_max_length(lengths):
        return sum(lengths)
    
    if not constraints:    # Default constraints
        constraints = [(-pi, pi)] * len(lengths)

    if len(lengths) == 1:
        # Case 1 joint
        return [atan2(y, x)]

    distance = dist((x, y), (0, 0))
    if distance > get_max_length(lengths):
        # Case too far to reach
        # print("Too far to reach")
        return [atan2(y, x)] + [0.0] * (len(lengths) - 1)
    
    # TODO: Accurate minimal length estimation
    
    if len(lengths) == 2:
        # Case 2 joints, reachable
        # print("Operating 2 joints")
        direction = LEFT if constraints[1][0] else RIGHT
        return ik_2links(x, y, lengths[0], lengths[1], direction)
    elif len(lengths) == 3:
        # Case 3 joints, reachable
        # print("Operating 3 joints")
        return ik_3links(x, y, lengths[0], lengths[1], lengths[2])
    else:
        raise ValueError("Only support 2 or 3 links")