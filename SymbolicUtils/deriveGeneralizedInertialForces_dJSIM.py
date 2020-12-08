from casadi import *
from SrdPy import SymbolicEngine


def deriveGeneralizedInertialForces_dJSIM(symbolicEngine:SymbolicEngine, jointSpaceInertiaMatrix):
    dJSIM = jacobian(jointSpaceInertiaMatrix, symbolicEngine.q) *symbolicEngine.v
    dJSIM = reshape(dJSIM, size(jointSpaceInertiaMatrix))

    return 0.5 * dJSIM * symbolicEngine.v