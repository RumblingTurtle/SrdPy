import numpy as np


def rotationMatrix3Dy(q):
    return [[np.cos(q), 0, np.sin(q)],
            [0, 1, 0],
            [-np.sin(q), 0, np.cos(q)]]