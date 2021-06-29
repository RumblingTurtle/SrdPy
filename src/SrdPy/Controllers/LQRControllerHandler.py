from control import lqr
from casadi import *
import numpy as np

class LQRControllerHandler():
    def __init__(self,stateSpaceHandler,controlInputStateSpaceHandler,
                 linearizedModelHandler,timeHandler,IKHandler,Q,R):
        self.stateSpaceHandler = stateSpaceHandler
        self.controlInputStateSpaceHandler = controlInputStateSpaceHandler
        self.linearizedModelHandler = linearizedModelHandler
        self.timeHandler = timeHandler
        self.IKHandler = IKHandler
        self.Q = Q
        self.R = R
        self.u = []

    def update(self):
        t = self.timeHandler.currentTime

        x,dx = self.controlInputStateSpaceHandler.getX_dx(t)

        A = self.linearizedModelHandler.getA()
        B = self.linearizedModelHandler.getB()

        K, S, CLP = lqr(A, B, self.Q, self.R)

        e = (self.stateSpaceHandler.x - x)

        u_FB = -K @ e

        u_FF = self.IKHandler.u

        self.u = u_FB + u_FF