import numpy as np
from numpy import cos,sin
from casadi import cos,sin

def rotationMatrix3Dx(q):
    return [[1, 0, 0],
            [0, cos(q), -sin(q)],
            [0, sin(q), cos(q)]]