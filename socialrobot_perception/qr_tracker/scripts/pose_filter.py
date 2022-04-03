#!/usr/bin/env python
import numpy as np
from geometry_msgs.msg import Pose


def dot(a, b):
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w

def close_quaternion(anchor, query):
    if dot(anchor, query) < 0.0:  # not close
        query.x *= -1.0
        query.y *= -1.0
        query.z *= -1.0
        query.w *= -1.0
    # close
    return query

def normalize_quaternion(orientation):        
    length = np.sqrt(
        orientation.x * orientation.x +
        orientation.y * orientation.y +
        orientation.z * orientation.z +
        orientation.w * orientation.w
    )
    n = 1.0 / length
    orientation.x *= n
    orientation.y *= n
    orientation.z *= n
    orientation.w *= n
    return orientation


def position_sq_dist(a, b):
    dx = a.x - b.x
    dy = a.y - b.y
    dz = a.z - b.z
    return dx*dx + dy*dy + dz*dz


def blend(prev_p, new_p):
    p = Pose()
    # Position
    sq_dist = position_sq_dist(prev_p.position, new_p.position)
    new_ratio = 0.04 * np.exp(-sq_dist)
    old_ratio = 1.0 - new_ratio
    p.position.x = old_ratio * prev_p.position.x + new_ratio * new_p.position.x
    p.position.y = old_ratio * prev_p.position.y + new_ratio * new_p.position.y
    p.position.z = old_ratio * prev_p.position.z + new_ratio * new_p.position.z
    # Orientation
    new_o = close_quaternion(prev_p.orientation, new_p.orientation)
    similarity = dot(prev_p.orientation, new_o)
    new_ratio = 0.08 * (similarity**8)
    old_ratio = 1.0 - new_ratio
    p.orientation.x = old_ratio * prev_p.orientation.x + new_ratio * new_o.x
    p.orientation.y = old_ratio * prev_p.orientation.y + new_ratio * new_o.y
    p.orientation.z = old_ratio * prev_p.orientation.z + new_ratio * new_o.z
    p.orientation.w = old_ratio * prev_p.orientation.w + new_ratio * new_o.w
    p.orientation = normalize_quaternion(p.orientation)
    return p
