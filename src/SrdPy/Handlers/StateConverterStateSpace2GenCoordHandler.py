from casadi import *
class StateConverterStateSpace2GenCoordHandler:
    def __init__(self,stateSpaceHandler):
        self.dofRobotStateSpace = stateSpaceHandler.dofStateSpace
        self.dofRobot = int(stateSpaceHandler.dofStateSpace/2)

        self.stateSpaceHandler = stateSpaceHandler
        self.q = None
        self.v = None
        self.a = None

    def getPositionVelocityAcceleration(self):
        return self.q, self.v, self.a

    def update(self):
        self.q = self.stateSpaceHandler.x[:self.dofRobot]
        self.v = self.stateSpaceHandler.x[self.dofRobot:]
        self.a = self.stateSpaceHandler.dx[self.dofRobot:]
