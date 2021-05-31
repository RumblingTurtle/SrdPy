from SrdPy import SrdMath
from SrdPy.LinksAndJoints import Joint
import numpy as np
class JointFloatingBase_XZ_plane(Joint):
    def __init__(self, name, childLink, parentLink, parentFollowerNumber,
                 usedGeneralizedCoordinates,usedControlInputs,defaultJointOrientation):
        self.type = "JointFloatingBase_XZ_plane"
        super(JointFloatingBase_XZ_plane, self).__init__(name, childLink, parentLink, parentFollowerNumber,
                                            usedGeneralizedCoordinates,usedControlInputs,defaultJointOrientation)
                                            
    @staticmethod
    def getJointInputsRequirements():
        return 6 

    def update(self,inputVector):
        q = inputVector[self.usedGeneralizedCoordinates]

        self.childLink.relativeOrientation =  SrdMath.rotationMatrix3Dy(q[0])
        
        self.childLink.absoluteBase = self.parentLink.absoluteFollower[self.parentFollowerNumber]+np.array([q[1],0,q[2]])
        self.childLink.absoluteOrientation = self.parentLink.absoluteOrientation@self.childLink.relativeOrientation

        rBaseToFollower = self.childLink.relativeFollower - np.matlib.repmat(self.childLink.relativeBase,self.childLink.relativeFollower.shape[0],1)
        rBaseToCoM = self.childLink.relativeCoM - self.childLink.relativeBase

        self.childLink.absoluteFollower = np.matlib.repmat(self.childLink.absoluteBase,self.childLink.relativeFollower.shape[0], 1) + (self.childLink.absoluteOrientation@rBaseToFollower.T).T

        self.childLink.absoluteCoM = self.childLink.absoluteBase + self.childLink.absoluteOrientation@rBaseToCoM
        
    def actionUpdate(self,inputVector):
        pass


