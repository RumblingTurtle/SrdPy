from casadi import *
from SrdPy import SymbolicEngine


def deriveGeneralizedInertialForces_dJSIM(symbolicEngine:SymbolicEngine, jointSpaceInertiaMatrix):
    dJSIM = MX.jacobian(jointSpaceInertiaMatrix, symbolicEngine.q) *symbolicEngine.v
    dJSIM = MX.reshape(dJSIM, MX.size(jointSpaceInertiaMatrix))

    return 0.5 * dJSIM * symbolicEngine.v