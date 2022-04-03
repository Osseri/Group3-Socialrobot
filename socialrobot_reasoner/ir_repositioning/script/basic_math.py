import numpy as np


def rotation_2d_about_z(rad):
    c, s = np.cos(rad), np.sin(rad)
    R = np.array((
        (c, -s),
        (s, c)
    ))
    return R


def rotation_3d_about_z(rad):
    c, s = np.cos(rad), np.sin(rad)
    R = np.array((
        (c, -s, 0),
        (s, c, 0),
        (0, 0, 1)
    ))
    return R
