from my_math import *

def ik(x, y, lengths):      # Result is absolute orientation in rad
    def ik_2links(x, y, l1, l2):
        cos_theta2 = constrain_sin_cos((x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2))
        sin_theta2 = sqrt(1 - cos_theta2**2)
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

    def get_min_length(lengths):
        n = len(lengths)
        total_sum = sum(lengths)
        # dynamic programming dp，dp[j] is bool: possiblility to find subset with sum = j
        dp = [False] * int(total_sum // 2 + 1)
        dp[0] = True  # subset with sum = 0 ([]) exist
        indices = [[] for _ in range(int(total_sum // 2 + 1))]
        for i, num in enumerate(lengths):
            for j in range(int(total_sum // 2), int(num - 1), -1):
                if dp[j - num]:
                    dp[j] = True
                    indices[j] = indices[j - num] + [i]
        # get the num closest to total_sum // 2
        for j in range(int(total_sum // 2), -1, -1):
            if dp[j]:
                subset_sum1 = j
                subset1_indices = indices[j]
                break
        subset_sum2 = total_sum - subset_sum1
        subset2_indices = [i for i in range(n) if i not in subset1_indices]
        if subset_sum1 > subset_sum2:   # Force subset1 shorter than subset2
            subset1_indices, subset2_indices = subset2_indices, subset1_indices
            subset_sum1, subset_sum2 = subset_sum2, subset_sum1
        return subset1_indices, subset2_indices, subset_sum2 - subset_sum1

    if len(lengths) == 1:
        # Case 1 joint
        print("Operating 1 joint")
        return [atan2(y, x)]

    distance = dist((x, y), (0, 0))
    if distance > get_max_length(lengths):
        # Case too far to reach
        print("Too far to reach")
        return [atan2(y, x)] + [0.0] * (len(lengths) - 1)
    # inward_joints, outward_joints, min_length = get_min_length(lengths)
    # if distance < min_length:
    #     # Case too close to reach
    #     print("Too close to reach")
    #     result = [None] * len(lengths)
    #     for i in inward_joints:
    #         result[i] = atan2(y, x) + pi
    #     for i in outward_joints:
    #         result[i] = atan2(y, x)
    #     return result
    
    if len(lengths) == 2:
        # Case 2 joints, reachable
        print("Operating 2 joints")
        return ik_2links(x, y, lengths[0], lengths[1])
    elif len(lengths) == 3:
        # Case 3 joints, reachable
        print("Operating 3 joints")
        return ik_3links(x, y, lengths[0], lengths[1], lengths[2])
    else:
        raise ValueError("Only support 2 or 3 links")