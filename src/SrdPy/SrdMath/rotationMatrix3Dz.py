import numpy as np
from numpy import cos,sin
from casadi import cos,sin

def rotationMatrix3Dz(q):
    return [[cos(q), -sin(q), 0],
            [sin(q), cos(q), 0],
            [0, 0, 1]]

def rotationMatrix3Dz_dq(q):
    return [[-sin(q), -cos(q), 0],
            [cos(q), -sin(q), 0],
            [0, 0, 0]]

def rotationMatrix3Dz_ddq(q):
    return [[-cos(q), sin(q), 0],
            [-sin(q), -cos(q), 0],
            [0, 0, 0]]

def rotationMatrix3Dz_dddq(q):
    return [[sin(q), cos(q), 0],
            [-cos(q), sin(q), 0],
            [0, 0, 0]]