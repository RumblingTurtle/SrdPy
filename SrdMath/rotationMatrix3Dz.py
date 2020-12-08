import numpy as np
from numpy import cos,sin
from casadi import cos,sin

def rotationMatrix3Dz(q):
    return [[cos(q), -sin(q), 0],
            [sin(q), cos(q), 0],
            [0, 0, 1]]