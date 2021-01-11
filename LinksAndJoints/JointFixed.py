from SrdPy.LinksAndJoints.Joint import Joint

class JointFixed(Joint):
    def __init__(self, name, childLink, parentLink, parentFollowerNumber,
                 usedGeneralizedCoordinates,usedControlInputs,defaultJointOrientation):
        self.type = "Fixed"
        super(JointFixed, self).__init__(name, childLink, parentLink, parentFollowerNumber,
                                            usedGeneralizedCoordinates,usedControlInputs,defaultJointOrientation)
    @staticmethod
    def getJointInputsRequirements():
        return 0

    def update(self, inputVector):
        self.childLink.relativeOrientation = self.defaultJointOrientation
        self.forwardKinematicsJointUpdate()