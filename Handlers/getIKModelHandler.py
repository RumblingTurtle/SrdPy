from casadi import *
from collections import namedtuple

from SrdPy.Handlers.Handler import Handler

class IKModelHandler(Handler):
    def __init__(self,description,dofRobot,dofTask):
        super(IKModelHandler,self).__init__()
        so_path = description["path"] + "/" + description["casadi_cCodeFilename"] + ".so"

        self.getTaskHandler = external(description["functionName_Task"], so_path)
        self.getJacobianHandler = external(description["functionName_TaskJacobian"], so_path)
        self.getJacobianDerivativeHandler = external(description["functionName_TaskJacobianDerivative"], so_path)
        self.dofRobot = dofRobot
        self.dofTask = dofTask


    def getTask(self, *args):
        return self.getTaskHandler(*args)


    def getJacobian(self, *args):
        return self.getJacobianHandler(*args)


    def getJacobianDerivative(self, *args):
        return self.getJacobianDerivativeHandler(*args)

def getIKModelHandler(description,dofRobot,dofTask):
    return IKModelHandler(description,dofRobot,dofTask)