#!/usr/bin/env python
from math import atan2
from geometry_msgs.msg import Vector3

def vector_add(v1, v2):
    v = Vector3()
    v.x = (v1.x + v2.x)
    v.y = (v1.y + v2.y)
    v.z = (v1.z + v2.z)
    return v

def vector_multiply(v1, scalar):
    v = Vector3()
    v.x = v1.x * scalar
    v.y = v1.y * scalar
    v.z = v1.z * scalar
    return v

def vector_mag(v):
    mag = pow(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2), 0.5)
    return mag

# Asngle assumes 2D vector, returns in radians
# Horizontal (1, 0) is an angle of 0, sweeps (+/-) going (CCW/CW)
def vector_ang(v):
    ang = -atan2(-v.y, v.x)
    return ang

def create_unit_vector(v1):
    v = Vector3()
    if vector_mag(v1) == 0:
        v.x = 1.0
    else:
        v.x = v1.x / vector_mag(v1)
        v.y = v1.y / vector_mag(v1)
    return v
