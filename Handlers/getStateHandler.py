import numpy as np

class StateHandler():
    def __init__(self,q,v,a,dofRobot):
        self.q = q
        self.v = v
        self.a = a
        self.dofRobot = dofRobot


    def getPositionVelocityAcceleration(self):
        return self.q, self.v, self.a

def getStateHandler(initialPosition, initialVelocity):
    return StateHandler(initialPosition,initialVelocity,np.full(initialPosition.shape, np.nan),initialPosition.shape[0])
