import numpy as np


def rotationMatrix3Dx(q):
    return [[1, 0, 0],
            [0, np.cos(q), -np.sin(q)],
            [0, np.sin(q), np.cos(q)]]