class DesiredStateHandler():
    def __init__(self,dofRobot,controlInputHandler,simulationHandler):
        self.dofRobot = dofRobot
        self.controlInputHandler = controlInputHandler
        self.simulationHandler = simulationHandler
        self.q = []
        self.v = []
        self.a = []

    def update(self):
        t = self.simulationHandler.currentTime
        self.q,self.v,self.a = self.controlInputHandler.getPositionVelocityAcceleration(t)

    def getPositionVelocityAcceleration(self,t):
        return self.q,self.v,self.a


def getDesiredStateHandler(controlInputHandler, simulationHandler):
    return DesiredStateHandler(controlInputHandler.dofRobot,controlInputHandler,simulationHandler)
