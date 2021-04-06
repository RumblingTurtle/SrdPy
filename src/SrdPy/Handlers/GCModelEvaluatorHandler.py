import numpy as np
from casadi import *
class GCModelEvaluatorHandler():
    def __init__(self,generalizedCoordinatesModel,stateHandler, usePinv=True):
        self.gCModel = generalizedCoordinatesModel
        self.stateHandler = stateHandler
        self.dofConfigurationSpaceRobot = self.gCModel.dofConfigurationSpaceRobot
        self.dofControl = self.gCModel.dofControl
        self.usePinv = usePinv

        self.lastUpdateQ = []
        self.lastUpdateV = []
        self.jointSpaceInertiaMatrix = []
        self.jointSpaceInertiaMatrixInverse = []
        self.biasVector = []
        self.controlMap = []

    def update(self):
        q,v,a =  self.stateHandler.getPositionVelocityAcceleration()
        self.jointSpaceInertiaMatrix = self.gCModel.getJointSpaceInertiaMatrix(np.array(DM(q)))
        self.biasVector = self.gCModel.getBiasVector(q, v)
        self.controlMap  = self.gCModel.getControlMap(q)

        if self.usePinv:
            self.jointSpaceInertiaMatrixInverse = pinv(self.jointSpaceInertiaMatrix)
        else:
            self.jointSpaceInertiaMatrixInverse = np.linalg.solve(self.jointSpaceInertiaMatrix,np.eye(self.jointSpaceInertiaMatrix))

        self.lastUpdateQ = q
        self.lastUpdateV = v
        
    def getJointSpaceInertiaMatrix(self,*args):
        return self.jointSpaceInertiaMatrix

    def getBiasVector(self,*args):
        return self.biasVector

    def getControlMap(self,*args):
        return self.controlMap

    def getJointSpaceInertiaMatrixInverse(self,*args):
        return self.jointSpaceInertiaMatrixInverse
