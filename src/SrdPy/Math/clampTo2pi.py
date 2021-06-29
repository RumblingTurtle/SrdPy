import numpy as np


def clampTo2pi(theta):
    return np.clip(theta, 0, 2 * np.pi)