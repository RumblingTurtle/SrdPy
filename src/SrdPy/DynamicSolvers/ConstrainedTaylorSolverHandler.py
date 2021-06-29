import numpy as np
from casadi import *

class ConstrainedTaylorSolverHandler():
    def __init__(self,stateHandler,controllerHandler,gcModelHandler,timeHandler,constraintsModel):
        self.stateHandler = stateHandler
        self.controllerHandler = controllerHandler
        self.gcModelHandler = gcModelHandler
        self.timeHandler = timeHandler
        self.constraintsModel = constraintsModel

    def update(self):
        dt = self.timeHandler.timeLog[self.timeHandler.currentIndex + 1]\
             - self.timeHandler.timeLog[self.timeHandler.currentIndex]

        q,v,a = self.stateHandler.getPositionVelocityAcceleration()

        n = self.gcModelHandler.dofConfigurationSpaceRobot
        k = self.constraintsModel.dofConstraint

        H = self.gcModelHandler.getJointSpaceInertiaMatrix(q)
        T = self.gcModelHandler.getControlMap(q)
        c = self.gcModelHandler.getBiasVector(q, v)
        
        F  = self.constraintsModel.getJacobian(DM(q))
        dF = self.constraintsModel.getJacobianDerivative(DM(q), DM(v))
        
        u = self.controllerHandler.u
        
        M = vertcat(horzcat(H, -F.T),horzcat(F,  SX.zeros((k,k))))
             
        
        vec = pinv(M) @ vertcat(T@u - c, -dF@v);
         
        
        a = vec[:n]
        
        v = v + dt * a
        q = q + dt * v + 0.5 * dt**2 * a
        

        self.stateHandler.q = q
        self.stateHandler.v = v
        self.stateHandler.a = a