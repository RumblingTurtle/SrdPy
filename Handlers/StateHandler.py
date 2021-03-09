import numpy as np

class StateHandler():
    def __init__(self,q,v):
        self.q = q
        self.v = v
        self.a = np.full(q.shape, np.nan)
        self.dofRobot = q.shape[0]


    def getPositionVelocityAcceleration(self):
        return self.q, self.v, self.a