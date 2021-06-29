from SrdPy import Math
from SrdPy.LinksAndJoints import Joint

class JointPivotXY(Joint):
    def __init__(self, name, childLink, parentLink, parentFollowerNumber,
                 usedGeneralizedCoordinates,usedControlInputs,defaultJointOrientation):
        self.type = "JointPivotXY"
        super(JointPivotXY, self).__init__(name, childLink, parentLink, parentFollowerNumber,
                                            usedGeneralizedCoordinates,usedControlInputs,defaultJointOrientation)
    @staticmethod
    def getJointInputsRequirements():
        return 1

    def update(self,inputVector):
        q = inputVector[self.usedGeneralizedCoordinates]
        Tx = Math.rotationMatrix3Dx(q[0])
        Ty = Math.rotationMatrix3Dy(q[1])
        
        self.childLink.relativeOrientation =  self.defaultJointOrientation@Ty@Tx

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

