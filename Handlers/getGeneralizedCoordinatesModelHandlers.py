from casadi import *
import numpy as np

class GeneralizedCoordinatesModelHandler():
    def __init__(self,description,usePinv):
        so_path = description["path"] + "/" + description["casadi_cCodeFilename"] + ".so"

        self.dofConfigurationSpaceRobot = description["dofConfigurationSpaceRobot"]
        self.dofControl = description["dofControl"]
        self.getJointSpaceInertiaMatrixHandler = external(description["functionName_H"], so_path)
        self.getBiasVectorHandler = external(description["functionName_c"], so_path)
        self.getControlMapHandler = external(description["functionName_T"], so_path)
        self.usePinv = usePinv

    def getJointSpaceInertiaMatrix(self, q):
        return self.getJointSpaceInertiaMatrixHandler(DM(q))

    def getJointSpaceInertiaMatrixInverse(self, q):
        H = self.getJointSpaceInertiaMatrixHandler(DM(q))

        if self.usePinv:
            return DM(pinv(H))
        else:
            return np.linalg.solve(H,np.eye(H.shape[0]))

    def getBiasVector(self, q,v):
        return DM(self.getBiasVectorHandler(DM(q),DM(v)))

    def getControlMap(self, q):
        return DM(self.getControlMapHandler(DM(q)))



def getGeneralizedCoordinatesModelHandlers(description,usePinv=False):
    return GeneralizedCoordinatesModelHandler(description,usePinv)