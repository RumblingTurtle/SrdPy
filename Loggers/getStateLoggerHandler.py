import numpy as np
class StateLoggerHandler():
    def __init__(self,stateHandler,simulationHandler,logAcceleration):
        self.stateHandler = stateHandler
        self.simulationHandler = simulationHandler
        self.logAcceleration = logAcceleration
        self.q = []
        self.v = []
        self.a = []

    def update(self):
        i = self.simulationHandler.currentIndex

        self.q.append(self.stateHandler.q)
        self.v.append(self.stateHandler.v)

        if self.logAcceleration:
            self.a.append(self.stateHandler.a)



def getStateLoggerHandler(stateHandler,simulationHandler,logAcceleration = True):
    return StateLoggerHandler(stateHandler,simulationHandler,logAcceleration)
