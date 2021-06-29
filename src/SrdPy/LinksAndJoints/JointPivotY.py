from SrdPy import Math
from SrdPy.LinksAndJoints import Joint
import numpy as np
class JointPivotY(Joint):
    def __init__(self, name, childLink, parentLink, parentFollowerNumber,
                 usedGeneralizedCoordinates,usedControlInputs,defaultJointOrientation):
        self.type = "PivotY"
        super(JointPivotY, self).__init__(name, childLink, parentLink, parentFollowerNumber,
                                            usedGeneralizedCoordinates,usedControlInputs,defaultJointOrientation)
    @staticmethod
    def getJointInputsRequirements():
        return 1

    def update(self, inputVector):
        if self.usedGeneralizedCoordinates[0]==0:
            coords = np.sign(self.usedGeneralizedCoordinates)
            coords[0]=1
            q = np.diag(coords)@inputVector[np.abs(self.usedGeneralizedCoordinates)]
        else:
            q = np.diag(np.sign(self.usedGeneralizedCoordinates))@inputVector[np.abs(self.usedGeneralizedCoordinates)]


        self.childLink.relativeOrientation = self.defaultJointOrientation@Math.rotationMatrix3Dy(q)

        self.forwardKinematicsJointUpdate()

    def actionUpdate(self,inputVector):
        u = inputVector[self.usedControlInputs]

        Child_T = self.childLink.absoluteOrientation

        torque_child  = Child_T.dot([0, u, 0])
        torque_parent = Child_T.dot([0, -u, 0])

        child_Jw = self.childLink.jacobianAngularVelocity
        parent_Jw = self.parentLink.jacobianAngularVelocity

        gen_force_child = child_Jw.dot(torque_child)
        gen_force_parent = parent_Jw.dot(torque_parent)

        return gen_force_child + gen_force_parent