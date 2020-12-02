from casadi import *
from SrdPy import SymbolicEngine


def deriveGeneralizedInertialForces_dH(symbolicEngine:SymbolicEngine, jointSpaceInertiaMatrix):

    H = jointSpaceInertiaMatrix
    q = symbolicEngine.q
    v = symbolicEngine.v

    dH = MX.jacobian(MX.reshape(H, len(v) * len(v), 1), q) * v
    dH = MX.reshape(dH, len(v), len(v))
    kineticEnergy = 0.5 * v.T * H * v
    return MX.reshape(dH * v, len(v), 1) - reshape(jacobian(kineticEnergy, q), len(v), 1)