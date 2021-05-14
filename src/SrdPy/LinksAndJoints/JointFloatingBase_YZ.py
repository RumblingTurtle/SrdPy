from SrdPy import SrdMath
from SrdPy.LinksAndJoints import Joint
import numpy as np
class JointFloatingBase_YZ(Joint):
    def __init__(self, name, childLink, parentLink, parentFollowerNumber,
                 usedGeneralizedCoordinates,usedControlInputs,defaultJointOrientation):
        self.type = "JointFloatingBase_YZ"
        super(JointFloatingBase_YZ, self).__init__(name, childLink, parentLink, parentFollowerNumber,
                                            usedGeneralizedCoordinates,usedControlInputs,defaultJointOrientation)
                                            
    @staticmethod
    def getJointInputsRequirements():
        return 3 

    def update(self,inputVector):
        q = inputVector[self.usedGeneralizedCoordinates]

        Tx = SrdMath.rotationMatrix3Dx(q[0])

        self.childLink.relativeOrientation =  np.array(Tx)

        self.childLink.absoluteBase = self.parentLink.absoluteFollower[self.parentFollowerNumber]+np.array([q[2],q[3]])
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


