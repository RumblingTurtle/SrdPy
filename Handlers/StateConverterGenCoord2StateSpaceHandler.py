from casadi import *
class StateConverterGenCoord2StateSpaceHandler():
    def __init__(self,handlerState):
        self.dofRobotStateSpace = 2*handlerState.dofRobot
        self.handlerState = handlerState
        self.x = None
        self.dx = None

    def getX_dx(self,*args):
        return self.x,self.dx

    def update(self):
        self.x = vertcat(self.handlerState.q,self.handlerState.v)
        self.dx = vertcat(self.handlerState.v,self.handlerState.a)
