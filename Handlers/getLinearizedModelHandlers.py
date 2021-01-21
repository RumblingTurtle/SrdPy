from casadi import *
from collections import namedtuple

def getLinearizedModelHandlers(description):
    so_path = description["path"] + "/" + description["casadi_cCodeFilename"] + ".so"
    fields = ["getA", "getB", "getC", "dofConfigurationSpaceRobot","dofStateSpaceRobot","dofControl"]

    handler = namedtuple("LinearizedModelHandlers", fields)

    dofConfigurationSpaceRobot = description["dofConfigurationSpaceRobot"]
    dofStateSpaceRobot = description["dofStateSpaceRobot"]
    dofControl = description["dofControl"]

    getA = external(description["functionName_A"], so_path)
    getB = external(description["functionName_B"], so_path)
    getC = None #external(description["functionName_c"], so_path)

    return handler(getA, getB, getC, dofConfigurationSpaceRobot, dofStateSpaceRobot, dofControl)