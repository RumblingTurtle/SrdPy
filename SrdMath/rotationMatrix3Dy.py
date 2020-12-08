import numpy as np
from numpy import cos,sin
from casadi import cos,sin

def rotationMatrix3Dy(q):
    return [[cos(q), 0, sin(q)],
            [0, 1, 0],
            [-sin(q), 0, cos(q)]]