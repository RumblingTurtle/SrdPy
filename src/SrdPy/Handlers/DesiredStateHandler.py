class DesiredStateHandler():
    def __init__(self,controlInputHandler, timeHandler):
       
        self.dofRobot = controlInputHandler.dofRobot
        self.controlInputHandler = controlInputHandler
        self.timeHandler = timeHandler
        self.q = []
        self.v = []
        self.a = []

    def update(self):
        t = self.timeHandler.currentTime
        self.q,self.v,self.a = self.controlInputHandler.getPositionVelocityAcceleration(t)

    def getPositionVelocityAcceleration(self,t):
        return self.q,self.v,self.a