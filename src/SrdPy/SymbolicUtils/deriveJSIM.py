from casadi import *
from SrdPy import SymbolicEngine


def deriveJSIM(symbolicEngine:SymbolicEngine):
    # H*ddq + c = T*u;        ddq = dv/dt; v = dq/dt;
    #
    # H = sum(  J'*m*J +Jw'*I*Jw  )
    H = SX.zeros(symbolicEngine.dof, symbolicEngine.dof)

    for link in symbolicEngine.linkArray:
        linkH = link.jacobianCenterOfMass.T @ link.mass @ link.jacobianCenterOfMass + \
            link.jacobianAngularVelocity.T @ link.inertia @ link.jacobianAngularVelocity
        H = H + linkH
        
    return H