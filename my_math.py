from math import sin, cos, sqrt

def constrain(data, constraint_low, constrain_high):
    if data > constrain_high:
        return constrain_high
    elif data < constraint_low:
        return constraint_low
    return data

def constrain_sin_cos(data):
    return constrain(data, -1, 1)

def dist(p1, p2):
    return sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def vec_add(v1, v2):
    return (v1[0] + v2[0], v1[1] + v2[1])

def vec_minus(v1, v2):
    return (v1[0] - v2[0], v1[1] - v2[1])

def vec_rotate(v, angle):
    return (v[0] * cos(angle) - v[1] * sin(angle), v[0] * sin(angle) + v[1] * cos(angle))

def vec_inverse(v):
    return (-v[0], -v[1])