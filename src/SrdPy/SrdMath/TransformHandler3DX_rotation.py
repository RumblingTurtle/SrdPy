from .rotationMatrix3Dx import *
from casadi import *
class TransformHandler3DX_rotation:
    def __init__(self):
        self._T = rotationMatrix3Dx
        self._dT = rotationMatrix3Dx_dq
        self._ddT = rotationMatrix3Dx_ddq
        self._dddT = rotationMatrix3Dx_dddq
    
    def T(self,q):
        return SX.zeros(3,3)+np.array(self._T(q))
    
    def dT(self,q):
        return SX.zeros(3,3)+np.array(self._dT(q))
        
    def ddT(self,q):
        return SX.zeros(3,3)+np.array(self._ddT(q))
        
    def dddT(self,q):
        return SX.zeros(3,3)+np.array(self._dddT(q))

    def angularVelocityJacobian(self):
        return SX.zeros(3)+np.array([1,0,0])
