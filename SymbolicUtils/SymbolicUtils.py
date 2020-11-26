import numpy as np
from casadi import *
from SrdPy import SymbolicEncoding


def deriveControlMap(symbolicEngine:SymbolicEncoding):
    return MX.eye(symbolicEngine.dof)

def deriveGeneralizedDissipativeForcesUniform(symbolicEngine:SymbolicEncoding,uniformCoefficient):
    return -uniformCoefficient * symbolicEngine.v

    # H * ddq + C * dq + g = T * u;
    #ddq = dv / dt;
    #v = dq / dt;
    #g = sum(J'*m*g )

def deriveGeneralizedGravitationalForces(symbolicEngine:SymbolicEncoding,gravitationalConstant=np.array([0,0,-9.8])):
    G = MX.zeros(symbolicEngine.dof, 1)

    for link in symbolicEngine.linkArray:
        G = G + link.Jacobian_CenterOfMass.T * link.Mass * gravitationalConstant

    G = MX.simplify(G)

    return G

    # H * ddq + c + g = T * u;
    #ddq = dv / dt;
    #v = dq / dt;
    # c = 0.5 * dH / dt * v

def deriveGeneralizedInertialForces_Christoffel(symbolicEngine:SymbolicEncoding,jointSpaceInertiaMatrix):

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

def deriveGeneralizedInertialForces_dH(symbolicEngine:SymbolicEncoding,jointSpaceInertiaMatrix):

    H = jointSpaceInertiaMatrix
    q = symbolicEngine.q
    v = symbolicEngine.v

    dH = MX.jacobian(MX.reshape(H, len(v) * len(v), 1), q) * v
    dH = MX.reshape(dH, length(v), len(v))
    kineticEnergy = 0.5 * v.T * H * v
    return MX.reshape(dH * v, len(v), 1) - reshape(jacobian(kineticEnergy, q), len(v), 1)

def deriveGeneralizedInertialForces_dJSIM(symbolicEngine:SymbolicEncoding,jointSpaceInertiaMatrix):
    dJSIM = MX.jacobian(jointSpaceInertiaMatrix, symbolicEngine.q) *symbolicEngine.v
    dJSIM = MX.reshape(dJSIM, MX.size(jointSpaceInertiaMatrix))

    return 0.5 * dJSIM * symbolicEngine.v

def deriveJacobiansForlinkArray(symbolicEngine:SymbolicEncoding):
    for link in symbolicEngine.linkArray:
        link.jacobianCenterOfMass = MX.jacobian(link.absoluteCoM, symbolicEngine.q)
        tVec = MX.reshape(link.absoluteOrientation, 9, 1)

        link.absoluteOrientationDerivative = MX.reshape(MX.jacobian(tVec, symbolicEngine.q) * symbolicEngine.v, [3, 3])

        # this is the so-called Poisson formula, that defines the
        # relations between the matrix of directional cosines and
        # the angular velocity in a skew-simmetric form (angular
        # velocity tensor)
        #
        # There are two eq. of interest for angular velocity
        # tensor;
        # first: (0)W = dT*T' where T is the matrix of directional
        # cosines for a local frame (basis), dT is its derivative,
        # (0)W denotes W expressed in the world frame;
        # second: (l)W = T'*dT, (1)W denotes W expressed in the local
        # frame. The following equality also holds:
        # (0)W = T*(l)W*T';
        #
        # For finding kinetic energy we need (l)W, because the tensor
        # of inertia will be expressed in the local frame.

        omega = link.absoluteOrientation.T * link.absoluteOrientationDerivative

        #The following gives us angular velocity w, expressed in the
        #local frame, see description of the angular velocity tensor.
        link.angularVelocity = [-omega[1, 2], omega[0, 2], -omega[0, 1]]

        link.jacobianAngularVelocity = MX.jacobian(link.angularVelocity, symbolicEngine.v)

# H*ddq + c = T*u;        ddq = dv/dt; v = dq/dt;
#
# H = sum(  J'*m*J +Jw'*I*Jw  )
def deriveJSIM(symbolicEngine:SymbolicEncoding):
    H = MX.zeros(symbolicEngine.dof, symbolicEngine.dof)

    for link in SymbolicEngine.linkArray:

        H = H + link.Jacobian_CenterOfMass.T * link.Mass * link.Jacobian_CenterOfMass + \
            link.Jacobian_AngularVelocity.T * link.Inertia * link.Jacobian_AngularVelocity

    return simplify(H)

def generateDynamicsLinearization(symbolicEngine:SymbolicEncoding,H,c,T):
    # H*ddq + c = T*u;        ddq = dv/dt; v = dq/dt;
    # x = [q; v]
    #
    #
    # f= ddq = inv(H) * (T*u - c)
    #
    # dx/dt = A*x+B*u+lc
    #
    # A = [0      I]
    #     [df/dq  df/dv  ]
    #
    # B = [0           ]
    #     [inv(H)*T    ]
    #
    # lc = [0                             ]
    #      [inv(H)*c - df/dq*q -  df/dv*v ]
    #
    # df / dq = d(inv(H))/dq * (T*u - c) + d(T*u - c)/dq
    # df / dq = inv(H) * dH/dq * inv(H) * (T*u - c) + d(T*u - c)/dq
    #
    # df / dv = inv(H)* d(T*u - c)/dv


    q = symbolicEngine.q
    v = symbolicEngine.v
    u = symbolicEngine.u

    n = symbolicEngine.dof
    m = len(symbolicEngine.u)

    iH = SX.sym('iH', [n, n])

    TCq = SX.jacobian(T*u+c, q)
    TCv = SX.jacobian(T*u+c, v)


    dfdq = -iH*SX.reshape(SX.jacobian(H, q)*(iH*(T*u+c)), n, n) + TCq

    dfdv = iH * TCv

    A = [[zeros(n, n), eye(n)], [dfdq, dfdv]]

    B = [[zeros(n, m)],[iH*T]]

    linear_c = [SX.zeros(n, 1),[iH*c - dfdq*q - dfdv*v]]

    return A,B,linear_c

