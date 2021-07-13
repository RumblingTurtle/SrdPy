from casadi import *
class LinearModelEvaluatorHandlerConstrained():
    def __init__(self,gcModel,linearizedModel,constraintsModel,stateHandler,controllerHandler):
        self.gcModel = gcModel
        self.linearizedModel = linearizedModel
        self.stateHandler = stateHandler
        self.controllerHandler = controllerHandler
        self.constraintsModel = constraintsModel
        
        self.dofControl = gcModel.dofControl
        self.dofConfigurationSpaceRobot = gcModel.dofConfigurationSpaceRobot
        self.dofRobotStateSpace = 2*self.dofConfigurationSpaceRobot
        self.dofConstraint = constraintsModel.dofConstraint

        self.lastUpdateQ = []
        self.lastUpdateV = []
        self.lastUpdateU = []

        self.A = []
        self.B = []
        self.c = []

    def update(self):
        q,v,a= self.stateHandler.getPositionVelocityAcceleration()
        q = DM(q)
        v = DM(v)
        u = DM(self.controllerHandler.u)

                    
        H  = self.gcModel.getJointSpaceInertiaMatrix(q)
        F  = self.constraintsModel.getJacobian(q)
        
        M = vertcat(horzcat(H, -F.T),
                horzcat(F, SX.zeros(self.dofConstraint,self.dofConstraint)))
        iM = pinv(M)
        
        self.A = self.linearizedModel.getA(q, v, u, iM)
        self.B = self.linearizedModel.getB(q, v,    iM)

        self.lastUpdateQ = q
        self.lastUpdateV = v
        self.lastUpdateU = u

    def getA(self):
        return self.A

    def getB(self):
        return self.B

    def getC(self):
        return self.c
