import numpy as np
class IDVanillaDesiredTrajectoryHandler():
    def __init__(self,controlInputHandler,gcModelHandler,timeHandler):
        self.controlInputHandler = controlInputHandler
        self.gcModelHandler = gcModelHandler
        self.timeHandler = timeHandler
        self.u = []

    def update(self):
        t = self.timeHandler.currentTime

        q,v,a = self.controlInputHandler.getPositionVelocityAcceleration(t)

        H = self.gcModelHandler.getJointSpaceInertiaMatrix(q)
        T = self.gcModelHandler.getControlMap(q)
        c = self.gcModelHandler.getBiasVector(q,v)

        self.u = np.linalg.pinv(T)@(H@a+c)