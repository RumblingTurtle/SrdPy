from casadi import *
from collections import namedtuple

def getGeneralizedCoordinatesModelHandlers(description):
    so_path = description["path"]+"/"+description["casadi_cCodeFilename"]+".so"
    handler = namedtuple("GeneralizedCoordinatesModelHandler","getJointSpaceInertiaMatrix getBiasVector getControlMap")
    getJointSpaceInertiaMatrix = external(description["functionName_H"],so_path)
    getBiasVector = external(description["functionName_c"],so_path)
    getControlMap = external(description["functionName_T"],so_path)

    return handler(getJointSpaceInertiaMatrix,getBiasVector,getControlMap)