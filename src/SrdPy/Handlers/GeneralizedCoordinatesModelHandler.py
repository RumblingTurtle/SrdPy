from casadi import *
import numpy as np
import scipy
class GeneralizedCoordinatesModelHandler():
    def __init__(self,description,usePinv=True):
        so_path = description["path"] + "/" + description["casadi_cCodeFilename"] + ".so"
        
        self.dofConfigurationSpaceRobot = description["dofConfigurationSpaceRobot"]
        self.dofControl = description["dofControl"]

        if description["useJIT"]:
            imp = Importer(description["path"] + "/" +description["casadi_cCodeFilename"]+".c","clang")
        else:
            imp = so_path
            
        self.getJointSpaceInertiaMatrixHandler = external(description["functionName_H"], imp)
        self.getBiasVectorHandler = external(description["functionName_c"], imp)
        self.getControlMapHandler = external(description["functionName_T"], imp)
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

        
    def getFirstOrderSystem_qv(self, x, u):
        q = x[:self.dofConfigurationSpaceRobot]
        v = x[self.dofConfigurationSpaceRobot+1:]

        iH = self.getJointSpaceInertiaMatrixInverse(q)
        c = self.getBiasVector(q, v)
        T = self.getControlMap(q)

        dx = vertcat(v,iH @ (T@u - c))
        return dx