import numpy as np

import SrdPy.SrdMath


def rotationMatrix3D(axisVector, theta):
    U = axisVector.dot(axisVector.T)
    Uc = SrdPy.SrdMath.crossProductMatrix3D.crossProductMatrix3D(axisVector)
    return np.cos(theta) * np.eye(3) + np.sin(theta) * Uc + (1 - np.cos(theta)) * U