from casadi import *
from collections import namedtuple

from SrdPy.Handlers.Handler import Handler

class IKModelHandler(Handler):
    def __init__(self,description,dofRobot,dofTask):
        super(IKModelHandler,self).__init__()
        so_path = description["path"] + "/" + description["casadi_cCodeFilename"] + ".so"

        self.taskHandler = external(description["functionName_Task"], so_path)
        self.jacobianHandler = external(description["functionName_TaskJacobian"], so_path)
        self.jacobianDerivativeHandler = external(description["functionName_TaskJacobianDerivative"], so_path)
        self.dofRobot = dofRobot
        self.dofTask = dofTask


    def getTask(self, q):
        return self.taskHandler(q)


    def getJacobian(self, q):
        return self.jacobianHandler(q)


    def getJacobianDerivative(self, q,v):
        return self.jacobianDerivativeHandler(q,v)

def getIKModelHandler(description,dofRobot,dofTask):
    return IKModelHandler(description,dofRobot,dofTask)