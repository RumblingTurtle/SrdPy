class LinearModelEvaluatorHandler():
    def __init__(self,gcModel,linearizedModel,stateHandler,controllerHandler,dofRobotStateSpace,dofControl,toEvaluateC):
        self.gcModel = gcModel
        self.linearizedModel = linearizedModel
        self.stateHandler = stateHandler
        self.controllerHandler = controllerHandler
        self.dofRobotStateSpace = dofRobotStateSpace
        self.dofControl = dofControl
        self.toEvaluateC = toEvaluateC

        self.lastUpdateQ = []
        self.lastUpdateV = []
        self.lastUpdateU = []

        self.A = []
        self.B = []
        self.c = []

    def update(self):
        q,v,a= self.stateHandler.getPositionVelocityAcceleration()

        u = self.controllerHandler.u

        iH = self.gcModel.getJointSpaceInertiaMatrixInverse(q)

        self.A = self.linearizedModel.getA(q, v, u, iH)
        self.B = self.linearizedModel.getB(q, v, iH)

        if self.toEvaluateC:
            self.c = self.linearizedModel.getC(q, v, u, iH)

        self.lastUpdateQ = q
        self.lastUpdateV = v
        self.lastUpdateU = u

    def getA(self):
        return self.A

    def getB(self):
        return self.B

    def getC(self):
        return self.c

def getLinearModelEvaluatorHandler(generalizedCoordinatesModel,linearizedModel,stateHandler,controllerHandler, toEvaluateC = True):
    dofRobotStateSpace = 2*generalizedCoordinatesModel.dofConfigurationSpaceRobot
    dofControl = generalizedCoordinatesModel.dofControl

    handler = LinearModelEvaluatorHandler(linearizedModel,stateHandler,controllerHandler,
                                          dofRobotStateSpace,dofControl,toEvaluateC)
    return handler
