from casadi import *
from collections import namedtuple

class ConstraintsModelHandler:
    def __init__(self,description,dofRobot):
        so_path = description["path"] + "/" + description["casadi_cCodeFilename"] + ".so"
        fields = ["getConstraint", "getJacobian", "getJacobianDerivative", "dofRobot","dofConstraint"]

        handler = namedtuple("GeneralizedCoordinatesModelHandler", fields)

        dofRobot = dofRobot
        if description["useJIT"]:
            imp = Importer(description["path"] + "/" +description["casadi_cCodeFilename"]+".c","clang")
        else:
            imp = so_path
            
        self.constraintHandler = external(description["functionName_Task"], imp)
        self.jacobianHandler = external(description["functionName_TaskJacobian"], imp)
        self.jacobianDerivativeHandler = external(description["functionName_TaskJacobianDerivative"], imp)
        self.dofConstraint = description["dofTask"]

    def getConstraint(self,**args):
        return self.constraintHandler(args)
    
    def getJacobian(self,q):
        return self.jacobianHandler(q)

    def getJacobianDerivative(self,q,v):
        return self.jacobianDerivativeHandler(q,v)