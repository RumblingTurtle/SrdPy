
from SrdPy import SrdMath
from SrdPy.LinksAndJoints import Joint

class JointPivotYZ(Joint):
    def __init__(self, name, childLink, parentLink, parentFollowerNumber,
                 usedGeneralizedCoordinates,usedControlInputs,defaultJointOrientation):
        self.type = "JointPivotYZ"
        super(JointPivotYZ, self).__init__(name, childLink, parentLink, parentFollowerNumber,
                                            usedGeneralizedCoordinates,usedControlInputs,defaultJointOrientation)
    @staticmethod
    def getJointInputsRequirements():
        return 1

    def update(self,inputVector):
        q = inputVector[self.usedGeneralizedCoordinates]
        Ty = SrdMath.rotationMatrix3Dy(q[0])
        Tz = SrdMath.rotationMatrix3Dz(q[1])
        
        self.childLink.relativeOrientation =  self.defaultJointOrientation@Tz@Ty

        self.forwardKinematicsJointUpdate()

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

