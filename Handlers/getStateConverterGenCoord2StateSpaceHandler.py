import numpy as np
class StateConverterGenCoord2StateSpaceHandler():
    def __init__(self,handlerState):
        self.dofRobotStateSpace = 2*handlerState.dofRobot
        self.handlerState = handlerState
        self.x = None
        self.dx = None

    def update(self):
        self.x = np.vstack((self.handlerState.q,self.handlerState.v))
        self.dx = np.vstack((self.handlerState.v,self.handlerState.a))

def getStateConverterGenCoord2StateSpaceHandler(handlerState):
    return StateConverterGenCoord2StateSpaceHandler(handlerState)
