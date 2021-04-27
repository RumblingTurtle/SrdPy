import imp
from SrdPy.LinksAndJoints.Joint import Joint
from SrdPy import SparseMatrix
from casadi import * 

class JointFixed(Joint):
    def __init__(self, name, childLink, parentLink, parentFollowerNumber,
                 usedGeneralizedCoordinates,usedControlInputs,defaultJointOrientation):
        super(JointFixed, self).__init__(name, childLink, parentLink, parentFollowerNumber,
                                            usedGeneralizedCoordinates,usedControlInputs,defaultJointOrientation)
        self.type = "Fixed"
        
    @staticmethod
    def getJointInputsRequirements():
        return 0

    def updateDerivatives(self,inputVector):
        self.childLink.absoluteOrientation_dTdq = SparseMatrix([],[],[3,3,self.childLink.dof],[1])
        self.childLink.absoluteOrientation_dTddq = SparseMatrix([],[],[3,3,self.childLink.dof,self.childLink.dof],[1,2])
        self.childLink.absoluteOrientation_dTdddq = SparseMatrix([],[],[3,3,self.childLink.dof,self.childLink.dof,self.childLink.dof],[1,2,3])

        self.childLink.relativeOrientation_dTdq = SparseMatrix([],[],[3,3,self.childLink.dof],[1])
        self.childLink.relativeOrientation_dTddq = SparseMatrix([],[],[3,3,self.childLink.dof,self.childLink.dof],[1,2])
        self.relativeOrientation_dTdddq = SparseMatrix([],[],[3,3,self.childLink.dof,self.childLink.dof,self.childLink.dof],[1,2,3])

        self.childLink.relativeAngularVelocityJacobian = SparseMatrix([],[],[3,1,self.childLink.dof],[1])
        self.childLink.H = SX.zeros(self.childLink.dof,self.childLink.dof)
        self.childLink.dH = SX.zeros(self.childLink.dof*self.childLink.dof*self.childLink.dof)
        self.childLink.ddH = SX.zeros(self.childLink.dof*self.childLink.dof*self.childLink.dof*self.childLink.dof)

    def update(self, inputVector):
        self.childLink.relativeOrientation = self.defaultJointOrientation
        self.forwardKinematicsJointUpdate()
        if self.childLink.dof==None:
            if type(inputVector)==SX:
                self.childLink.dof = inputVector.size()[0]
            else:
                self.childLink.dof = inputVector.shape[0]
            
            self.updateDerivatives(inputVector)