from my_math import *
from min_distance import calculate_minimum_distance

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

        print("2 way ik,", theta1, theta2)
        
        return [theta1, theta2]

    def ik_3links(x, y, l1, l2, l3):
        cos_theta3 = constrain_sin_cos((x**2 + y**2 - l1**2 - l2**2 - l3**2) / (2 * l1 * l2 * l3))
        sin_theta3 = sqrt(1 - cos_theta3**2)
        theta3 = atan2(sin_theta3, cos_theta3)
        
        k1 = l1 + l2 * cos_theta3
        k2 = l2 * sin_theta3
        theta2 = atan2(y, x) - atan2(k2, k1)
        
        cos_theta1 = constrain_sin_cos((x - l3 * cos(theta2 + theta3)) / l1)
        sin_theta1 = constrain_sin_cos((y - l3 * sin(theta2 + theta3)) / l1)
        theta1 = atan2(sin_theta1, cos_theta1)

        print("3 way ik,", theta1, theta2, theta3)
        
        return [theta1, theta2, theta3]

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
        print("Too far to reach")
        return [atan2(y, x)] + [0.0] * (len(lengths) - 1)
    
    # TODO: Accurate minimal length estimation
    
    if len(lengths) == 2:
        # Case 2 joints, reachable
        print("Operating 2 joints")
        direction = LEFT if constraints[1][0] else RIGHT
        return ik_2links(x, y, lengths[0], lengths[1], direction)
    elif len(lengths) == 3:
        # Case 3 joints, reachable
        print("Operating 3 joints")
        return ik_3links(x, y, lengths[0], lengths[1], lengths[2])
    else:
        raise ValueError("Only support 2 or 3 links")