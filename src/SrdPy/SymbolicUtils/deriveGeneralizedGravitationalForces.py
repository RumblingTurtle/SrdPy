import numpy as np
from casadi import *

from SrdPy import SymbolicEngine


# H * ddq + C * dq + g = T * u;
# ddq = dv / dt;
# v = dq / dt;
# g = sum(J'*m*g )

def deriveGeneralizedGravitationalForces(symbolicEngine:SymbolicEngine, gravitationalConstant=np.array([0, 0, -9.8])):
    G = SX.zeros(symbolicEngine.dof, 1)

    for link in symbolicEngine.linkArray:
        G = G +link.mass * link.jacobianCenterOfMass.T @ gravitationalConstant

    G = simplify(G)

    return G