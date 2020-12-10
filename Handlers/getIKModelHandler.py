from casadi import *
from collections import namedtuple

def getIKModelHandler(description,dofRobot,dofTask):
    so_path = description["path"] + "/" + description["casadi_cCodeFilename"] + ".so"
    fields = ["getTask", "getJacobian", "getJacobianDerivative", "dofRobot","dofTask"]

    handler = namedtuple("GeneralizedCoordinatesModelHandler", fields)

    getTask = external(description["functionName_Task"], so_path)
    getJacobian = external(description["functionName_TaskJacobian"], so_path)
    getJacobianDerivative = external(description["functionName_TaskJacobianDerivative"], so_path)

    return handler(getTask , getJacobian, getJacobianDerivative, dofRobot,dofTask)