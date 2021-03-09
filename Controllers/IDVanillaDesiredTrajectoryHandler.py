import numpy as np
class IDVanillaDesiredTrajectoryHandler():
    def __init__(self,controlInputHandler,gcModelHandler,simulationHandler):
        self.controlInputHandler = controlInputHandler
        self.gcModelHandler = gcModelHandler
        self.simulationHandler = simulationHandler
        self.u = []

    def update(self):
        t = self.simulationHandler.currentTime

        q,v,a = self.controlInputHandler.getPositionVelocityAcceleration(t)

        H = self.gcModelHandler.getJointSpaceInertiaMatrix(q)
        T = self.gcModelHandler.getControlMap(q)
        c = self.gcModelHandler.getBiasVector(q,v)

        self.u = np.linalg.pinv(T)@(H@a+c)