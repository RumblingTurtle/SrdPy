import numpy as np
import scipy

#Mistry, M., Buchli, J. and Schaal, S., 2010, May. Inverse dynamics control of floating base 
#systems using orthogonal decomposition. In 2010 IEEE international conference on robotics 
#and automation (pp. 3406-3412). IEEE.
class InverseDynamicsConstrained_QR():
    def __init__(self,controlInputHandler,constraintsModel,gcModelHandler,simulationHandler):
        self.controlInputHandler = controlInputHandler
        self.constraintsModel = constraintsModel
        self.gcModelHandler = gcModelHandler
        self.simulationHandler = simulationHandler
        self.u = []
        self.lambd = []

    def update(self):
        t = self.simulationHandler.currentTime

        n = self.gcModelHandler.dofConfigurationSpaceRobot
        k = self.constraintsModel.dofConstraint

        q,v,a = self.controlInputHandler.getPositionVelocityAcceleration(t)

        H = self.gcModelHandler.getJointSpaceInertiaMatrix(q)
        T = self.gcModelHandler.getControlMap(q)
        c = self.gcModelHandler.getBiasVector(q,v)

        F = self.constraintsModel.getJacobian(q)
        Q, R = scipy.linalg.qr(F.T)

        R_c = R[:k] 

        I = np.eye(n)
        I_c = I[:k]
        I_u = I[k:]

        u_FF = np.linalg.pinv(I_u@Q.T@T)@I_u@Q.T@(H@a+c)
        lambd = np.linalg.pinv(R_c) @ I_c @ Q.T @ (H@a+ c - T@u_FF)

        self.u = u_FF
        self.lambd = lambd