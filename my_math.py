from math import pi, sin, cos, sqrt, atan2

def constrain_sin_cos(data):
    if data > 1:
        return 1
    elif data < -1:
        return -1
    return data

def dist(p1, p2):
    return sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def vec_add(v1, v2):
    return (v1[0] + v2[0], v1[1] + v2[1])

def vec_minus(v1, v2):
    return (v1[0] - v2[0], v1[1] - v2[1])

def vec_rotate(v, angle):
    return (v[0] * cos(angle) - v[1] * sin(angle), v[0] * sin(angle) + v[1] * cos(angle))