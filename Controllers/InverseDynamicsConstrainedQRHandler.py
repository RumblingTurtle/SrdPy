import numpy as np
from scipy.linalg import orth
class InverseDynamicsConstrainedQRHandler:
    def __init__(self, inverseDynamicsHandler, controlInputHandler, gcModelHandler, constraintsModelHandler, simulationHandler):
        self.inverseDynamicsHandler = inverseDynamicsHandler
        self.controlInputHandler = controlInputHandler
        self.gcModelHandler = gcModelHandler
        self.constraintsModelHandler = constraintsModelHandler
        self.simulationHandler = simulationHandler
    
    def update(self):
        t = self.simulationHandler.currentTime
        n = self.gcModelHandler.dofConfigurationSpaceRobot
        k = self.constraintsModelHandler.dofConstraint

        desired_q, desired_v, desired_a = self.controlInputHandler.getPositionVelocityAcceleration(t)

        H = self.gcModelHandler.getJointSpaceInertiaMatrixInverse(desired_q)
        T = self.gcModelHandler.getControlMap(desired_q)
        c = self.gcModelHandler.getBiasVector(desired_q, desired_v)
        
        F = self.constraintsModelHandler.getJacobian(desired_q)
        
        F = orth(F.T)
        F = F.T

        Q, R = np.linalg.qr(F.T)
        R_c = R[:k]
        
        I = np.eye(n)
        I_c = I[:k]
        I_u = I[k:]
        
        u_FF = np.linalg.pinv(I_u @ Q.T @ T) @ I_u @ Q.T @ (H@desired_a + c)
        
        lambd = np.linalg.pinv(R_c) @ I_c @ Q.T @ (H@desired_a + c - T@u_FF)
        

        self.inverseDynamicsHandler.u = u_FF
        
        self.inverseDynamicsHandler.lambd = lambd