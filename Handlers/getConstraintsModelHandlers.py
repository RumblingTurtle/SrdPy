from casadi import *
from collections import namedtuple

def getConstraintsModelHandlers(description,dofRobot):
    so_path = description["path"] + "/" + description["casadi_cCodeFilename"] + ".so"
    fields = ["getConstraint", "getJacobian", "getJacobianDerivative", "dofRobot","dofConstraint"]

    handler = namedtuple("GeneralizedCoordinatesModelHandler", fields)

    dofRobot = dofRobot

    getConstraint = external(description["functionName_Task"], so_path)
    getJacobian = external(description["functionName_TaskJacobian"], so_path)
    getJacobianDerivative = external(description["functionName_TaskJacobianDerivative"], so_path)
    dofConstraint = description["dofTask"]

    return handler(getConstraint, getJacobian, getJacobianDerivative, dofRobot, dofConstraint)