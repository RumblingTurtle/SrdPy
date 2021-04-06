from casadi import *

from SrdPy import SymbolicEngine


def deriveGeneralizedInertialForces_Christoffel(symbolicEngine:SymbolicEngine, jointSpaceInertiaMatrix):

    H = jointSpaceInertiaMatrix
    q = symbolicEngine.q
    v = symbolicEngine.v
    n = jointSpaceInertiaMatrix.shape[0]

    forces = MX.zeros(n, 1);

    for i in range(n):
        for j in range(n):
            for k in range(n):
                G = 0.5 * jacobian(H(i, j), q(k)) + 0.5 * jacobian(H(i, k), q(j)) - 0.5 * jacobian(H(k, j), q(i));

                forces[i] = forces[i] + G * v[j] * v[k]

    return forces