import numpy as np
from numpy import cos,sin
from casadi import cos,sin

def rotationMatrix3Dy(q):
    return [[cos(q), 0, sin(q)],
            [0, 1, 0],
            [-sin(q), 0, cos(q)]]

def rotationMatrix3Dy_dq(q):
    return [[-sin(q), 0, cos(q)],
            [0, 0, 0],
            [-cos(q), 0, -sin(q)]]

def rotationMatrix3Dy_ddq(q):
    return [[-cos(q), 0, -sin(q)],
            [0, 0, 0],
            [sin(q), 0, -cos(q)]]

def rotationMatrix3Dy_dddq(q):
    return [[sin(q), 0, -cos(q)],
            [0, 0, 0],
            [cos(q), 0, sin(q)]]