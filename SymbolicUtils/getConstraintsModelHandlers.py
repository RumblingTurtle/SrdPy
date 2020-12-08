from casadi import *
from collections import namedtuple

def getConstraintsModelHandlers(description):
    so_path = description["path"] + "/" + description["casadi_cCodeFilename"] + ".so"
    fields = ["getConstraint", "getJacobian", "getJacobianDerivative", "dofRobot"]

    handler = namedtuple("GeneralizedCoordinatesModelHandler", fields)

    dofRobot = description["dofRobot"]
    getConstraint = external(description["functionName_Task"], so_path)
    getJacobian = external(description["functionName_TaskJacobian"], so_path)
    getJacobianDerivative = external(description["functionName_TaskJacobianDerivative"], so_path)

    return handler(getConstraint, getJacobian, getJacobianDerivative, dofRobot)