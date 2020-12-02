import numpy as np


def rotationMatrix2D(q):
    return [[np.cos(q), -np.sin(q)],
            [np.sin(q), np.cos(q)]]