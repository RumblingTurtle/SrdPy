from casadi import *
from SrdPy import SymbolicEngine


def deriveJacobiansForlinkArray(symbolicEngine:SymbolicEngine):
    for link in symbolicEngine.linkArray:
        link.jacobianCenterOfMass = jacobian(link.absoluteCoM, symbolicEngine.q)
        tVec = MX.reshape(link.absoluteOrientation, (9, 1))

        tVecJacobian = jacobian(tVec, symbolicEngine.q)


        link.absoluteOrientationDerivative = MX.reshape(dot(transpose(tVecJacobian),transpose(symbolicEngine.v)), [3, 3])

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

        link.jacobianAngularVelocity = jacobian(link.angularVelocity, symbolicEngine.v)