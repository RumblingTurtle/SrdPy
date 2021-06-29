import numpy as np
from casadi import *
class StateLoggerHandler():
    def __init__(self,stateHandler,timeHandler,logAcceleration = True):
        self.stateHandler = stateHandler
        self.timeHandler = timeHandler
        self.logAcceleration = logAcceleration
        self.q = []
        self.v = []
        self.a = []

    def update(self):
        i = self.timeHandler.currentIndex

        self.q.append(np.array(DM(self.stateHandler.q)).T[0])
        self.v.append(np.array(DM(self.stateHandler.v)).T[0])

        if self.logAcceleration:
            self.a.append(np.array(DM(self.stateHandler.a)).T[0])
