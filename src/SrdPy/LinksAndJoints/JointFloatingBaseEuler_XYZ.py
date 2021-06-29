from SrdPy import Math
from SrdPy.LinksAndJoints import Joint
import numpy as np
class JointFloatingBaseEuler_XYZ(Joint):
    def __init__(self, name, childLink, parentLink, parentFollowerNumber,
                 usedGeneralizedCoordinates,usedControlInputs,defaultJointOrientation):
        self.type = "JointFloatingBaseEuler_XYZ"
        super(JointFloatingBaseEuler_XYZ, self).__init__(name, childLink, parentLink, parentFollowerNumber,
                                            usedGeneralizedCoordinates,usedControlInputs,defaultJointOrientation)
                                            
    @staticmethod
    def getJointInputsRequirements():
        return 6 

    def update(self,inputVector):
        q = inputVector[self.usedGeneralizedCoordinates]

        Tx = Math.rotationMatrix3Dx(q[0])
        Ty = Math.rotationMatrix3Dy(q[1])
        Tz = Math.rotationMatrix3Dz(q[2])

        self.childLink.relativeOrientation =  np.array(Tz)@np.array(Ty)@np.array(Tx)

        self.childLink.absoluteBase = self.parentLink.absoluteFollower[self.parentFollowerNumber]+np.array([q[3],q[4],q[5]])
        self.childLink.absoluteOrientation = self.parentLink.absoluteOrientation@self.childLink.relativeOrientation
      
        rBaseToFollower = self.childLink.relativeFollower - np.matlib.repmat(self.childLink.relativeBase,self.childLink.relativeFollower.shape[0],1)
        rBaseToCoM = self.childLink.relativeCoM - self.childLink.relativeBase

        self.childLink.absoluteFollower = np.matlib.repmat(self.childLink.absoluteBase,self.childLink.relativeFollower.shape[0], 1) + (self.childLink.absoluteOrientation@rBaseToFollower.T).T

        self.childLink.absoluteCoM = self.childLink.absoluteBase + self.childLink.absoluteOrientation@rBaseToCoM
        
    def actionUpdate(self,inputVector):
        u = inputVector[self.usedControlInputs]

        Child_T = self.childLink.absoluteOrientation

        torque_child  = Child_T.dot([u, 0, 0])
        torque_parent = Child_T.dot([-u, 0, 0])

        child_Jw = self.childLink.jacobianAngularVelocity
        parent_Jw = self.parentLink.jacobianAngularVelocity

        gen_force_child = child_Jw.dot(torque_child)
        gen_force_parent = parent_Jw.dot(torque_parent)

        return gen_force_child + gen_force_parent


