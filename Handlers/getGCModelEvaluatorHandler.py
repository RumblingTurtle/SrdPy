import numpy as np
class GCModelEvaluatorHandler():
    def __init__(self,generalizedCoordinatesModel,stateHandler, usePinv):
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
        self.jointSpaceInertiaMatrix = self.gCModel.getJointSpaceInertiaMatrix(q)
        self.biasVector = self.gCModel.getBiasVector(q, v)
        self.controlMap  = self.gCModel.getControlMap(q)

        if self.usePinv:
            self.jointSpaceInertiaMatrixInverse = pinv(self.jointSpaceInertiaMatrix)
        else:
            self.jointSpaceInertiaMatrixInverse = np.linalg.solve(self.jointSpaceInertiaMatrix,np.eye(self.jointSpaceInertiaMatrix))

        self.lastUpdateQ = q
        self.lastUpdateV = v
        
    def get_joint_space_inertia_matrix(self):
        return self.jointSpaceInertiaMatrix

    def getBiasVector(self):
        return self.biasVector

    def getControlMap(self):
        return self.controlMap

    def get_joint_space_inertia_matrix_inverse(self):
        return self.jointSpaceInertiaMatrixInverse

def getGCModelEvaluatorHandler(generalizedCoordinatesModel,stateHandler,usePinv=True):
    return GCModelEvaluatorHandler(generalizedCoordinatesModel,stateHandler,usePinv)