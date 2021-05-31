import numpy as np

class ComputedTorqueController():
    def __init__(self, stateHandler, controlInputHandler, gcModelHandler, simulationHandler, IKHandler,kP,kD):
        self.u = []
        self.stateHandler = stateHandler
        self.IKHandler = IKHandler
        self.controlInputHandler = controlInputHandler
        self.gcModelHandler = gcModelHandler
        self.simulationHandler = simulationHandler
        self.kP = kP
        self.kD = kD


    def update(self):
        t = self.simulationHandler.currentTime

        q,v,a = self.controlInputHandler.getPositionVelocityAcceleration(t)

        H = self.gcModelHandler.getJointSpaceInertiaMatrix(self.stateHandler.q)
        T = self.gcModelHandler.getControlMap(self.stateHandler.q)

        e = q - self.stateHandler.q
        de = v - self.stateHandler.v
        
        u_FB = np.linalg.pinv(T) @ (H @ (self.kP @ e + self.kD @ de))

        u_FF = self.IKHandler.u

        self.u = u_FB + u_FF