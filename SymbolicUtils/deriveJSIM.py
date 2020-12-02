from casadi import *
from SrdPy import SymbolicEngine


def deriveJSIM(symbolicEngine:SymbolicEngine):
    # H*ddq + c = T*u;        ddq = dv/dt; v = dq/dt;
    #
    # H = sum(  J'*m*J +Jw'*I*Jw  )
    H = MX.zeros(symbolicEngine.dof, symbolicEngine.dof)

    for link in SymbolicEngine.linkArray:

        H = H + link.Jacobian_CenterOfMass.T * link.Mass * link.Jacobian_CenterOfMass + \
            link.Jacobian_AngularVelocity.T * link.Inertia * link.Jacobian_AngularVelocity

    return simplify(H)