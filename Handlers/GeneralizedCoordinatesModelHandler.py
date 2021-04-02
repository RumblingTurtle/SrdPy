from casadi import *
import numpy as np
import scipy
class GeneralizedCoordinatesModelHandler():
    def __init__(self,description,usePinv=True):
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
        H = np.array(self.getJointSpaceInertiaMatrixHandler(DM(q)))

        if self.usePinv:
            return np.linalg.pinv(H)
        else:
            return np.linalg.solve(H,np.eye(H.shape[0]))

    def getBiasVector(self, q,v):
        return DM(self.getBiasVectorHandler(DM(q),DM(v)))

    def getControlMap(self, q):
        return np.array(self.getControlMapHandler(DM(q)))