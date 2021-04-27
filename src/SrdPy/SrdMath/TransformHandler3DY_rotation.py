from .rotationMatrix3Dy import *
from casadi import *
class TransformHandler3DY_rotation:
    def __init__(self):
        self._T = rotationMatrix3Dy
        self._dT = rotationMatrix3Dy_dq
        self._ddT = rotationMatrix3Dy_ddq
        self._dddT = rotationMatrix3Dy_dddq
    
    def T(self,q):
        return SX.zeros(3,3)+np.array(self._T(q))
    
    def dT(self,q):
        return SX.zeros(3,3)+np.array(self._dT(q))
        
    def ddT(self,q):
        return SX.zeros(3,3)+np.array(self._ddT(q))
        
    def dddT(self,q):
        return SX.zeros(3,3)+np.array(self._dddT(q))

    def angularVelocityJacobian(self):
        return SX.zeros(3)+np.array([0,1,0])
