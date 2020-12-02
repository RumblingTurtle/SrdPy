import numpy as np
from casadi import *

from SrdPy import SymbolicEngine


# H * ddq + C * dq + g = T * u;
# ddq = dv / dt;
# v = dq / dt;
# g = sum(J'*m*g )

def deriveGeneralizedGravitationalForces(symbolicEngine:SymbolicEngine, gravitationalConstant=np.array([0, 0, -9.8])):
    G = MX.zeros(symbolicEngine.dof, 1)

    for link in symbolicEngine.linkArray:
        G = G + link.Jacobian_CenterOfMass.T * link.Mass * gravitationalConstant

    G = MX.simplify(G)

    return G