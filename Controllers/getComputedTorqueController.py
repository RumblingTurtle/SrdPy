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

        H = self.gcModelHandler.GetJointSpaceInertiaMatrix(self.stateHandler.q)
        T = self.gcModelHandler.getControlMap(self.stateHandler.q)

        e = np.reshape((q - self.stateHandler.q), [], 1)
        de = np.reshape((v - self.stateHandler.v), [], 1)
        
        u_FB = np.linalg.pinv(T) @ (H @ (self.kP @ e + self.kD @ de))

        u_FF = self.IKHandler.u

        self.u = u_FB + u_FF

def getComputedTorqueController(stateHandler, controlInputHandler, gcModelHandler, simulationHandler, IKHandler,kP,kD):
    return ComputedTorqueController(stateHandler, controlInputHandler, gcModelHandler, simulationHandler, IKHandler,kP,kD)
