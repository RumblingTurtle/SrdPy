from SrdPy import SrdMath
from SrdPy.LinksAndJoints.SrdJoint import SrdJoint

class SrdJointPivotZ(SrdJoint):
    def __init__(self, name, childLink, parentLink, parentFollowerNumber,
                 usedGeneralizedCoordinates, usedControlInputs, defaultJointOrientation):
        self.type = "PivotZ"
        super(SrdJointPivotZ, self).__init__(name, childLink, parentLink, parentFollowerNumber,
                                             usedGeneralizedCoordinates, usedControlInputs, defaultJointOrientation)
    @staticmethod
    def getJointInputsRequirements():
        return 1

    def update(self, inputVector):
        q = inputVector[self.usedGeneralizedCoordinates]

        self.childLink.relativeOrientation = self.defaultJointOrientation@SrdMath.rotationMatrix3Dz(q)

        import numpy as np
        print("Pivot z")
        print(self.childLink.relativeOrientation)
        print(np.array(SrdMath.rotationMatrix3Dz(q)))
        print(self.defaultJointOrientation)
        print()
        self.forwardKinematicsJointUpdate()

    def actionUpdate(self,inputVector):
        u = inputVector[self.usedControlInputs]

        Child_T = self.childLink.absoluteOrientation

        torque_child  = Child_T.dot([0, 0, u])
        torque_parent = Child_T.dot([0, 0, -u])

        child_Jw = self.childLink.jacobianAngularVelocity
        parent_Jw = self.parentLink.jacobianAngularVelocity

        gen_force_child = child_Jw.dot(torque_child)
        gen_force_parent = parent_Jw.dot(torque_parent)

        return gen_force_child + gen_force_parent