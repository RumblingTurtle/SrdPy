from casadi import *
from SrdPy import SymbolicEngine


def deriveGeneralizedInertialForces_dH(symbolicEngine:SymbolicEngine, jointSpaceInertiaMatrix):
    H = jointSpaceInertiaMatrix
    q = symbolicEngine.q
    v = symbolicEngine.v

    #Uses twice the space for both linearization and GC model generated code
    #dH = jacobian(reshape(H, v.shape[0]**2, 1), q) @ v 
    dH = jtimes(reshape(H, v.shape[0]**2, 1), q,v)
    dH = reshape(dH, v.shape[0], v.shape[0])
    kineticEnergy = 0.5 * v.T @ H @ v
    return reshape(dH @ v, v.shape[0], 1) - reshape(jacobian(kineticEnergy, q), v.shape[0], 1),dH