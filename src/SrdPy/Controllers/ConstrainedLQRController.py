from control import lqr
from casadi import *
import scipy
import numpy as np

class ConstrainedLQRController():
    def __init__(self,stateHandler,stateSpaceHandler,controlInputStateSpaceHandler,
                 linearizedModelHandler,constraintsHandler,timeHandler,inverseDynamicsHandler,Q,R):
        self.stateHandler = stateHandler
        self.stateSpaceHandler = stateSpaceHandler
        self.controlInputStateSpaceHandler = controlInputStateSpaceHandler
        self.linearizedModelHandler = linearizedModelHandler
        self.constraintsHandler = constraintsHandler
        self.timeHandler = timeHandler
        self.inverseDynamicsHandler = inverseDynamicsHandler
        self.Q = Q
        self.R = R
        self.u = []

    def update(self):
        t = self.timeHandler.currentTime

        x,dx = self.controlInputStateSpaceHandler.getX_dx(t)

        q,v,a = self.stateHandler.getPositionVelocityAcceleration()
        
        A = self.linearizedModelHandler.getA()
        B = self.linearizedModelHandler.getB()


        F = self.constraintsHandler.getJacobian(DM(q))
        dF = self.constraintsHandler.getJacobianDerivative(DM(q),DM(v))
        
        G = horzcat(F,dF)

        N = scipy.linalg.null_space(G)

        An = N.T@A@N
        Bn = N.T@B.T
        Qn = N.T@self.Q@N

        Kn, S, CLP = lqr(An, Bn, Qn, self.R)

        K = Kn@N.T
        
        e = (self.stateSpaceHandler.x - x)

        u_FB = -K @ e

        u_FF = self.inverseDynamicsHandler.u

        self.u = u_FB + u_FF