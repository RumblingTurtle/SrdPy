from casadi import *
from collections import namedtuple

def getGeneralizedCoordinatesModelHandlers(description):
    so_path = description["path"]+"/"+description["casadi_cCodeFilename"]+".so"
    fields = ["getJointSpaceInertiaMatrix","getBiasVector","getControlMap","dofConfigurationSpaceRobot","dofControl"]

    handler = namedtuple("GeneralizedCoordinatesModelHandler",fields)

    dofConfigurationSpaceRobot = description["dofConfigurationSpaceRobot"]
    dofControl = description["dofControl"]
    getJointSpaceInertiaMatrix = external(description["functionName_H"],so_path)
    getBiasVector = external(description["functionName_c"],so_path)
    getControlMap = external(description["functionName_T"],so_path)

    return handler(getJointSpaceInertiaMatrix,getBiasVector,getControlMap,dofConfigurationSpaceRobot,dofControl)