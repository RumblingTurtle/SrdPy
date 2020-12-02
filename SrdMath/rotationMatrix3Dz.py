import numpy as np


def rotationMatrix3Dz(q):
    return [[np.cos(q), -np.sin(q), 0],
            [np.sin(q), np.cos(q), 0],
            [0, 0, 1]]