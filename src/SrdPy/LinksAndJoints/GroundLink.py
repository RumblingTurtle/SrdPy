import imp
import numpy as np
from SrdPy.LinksAndJoints import Link
from SrdPy import SparseMatrix
from casadi import *

class GroundLink(Link):
    def __init__(self):
        super(GroundLink, self).__init__(name='Ground', order=0, relativeBase= np.array([0,0,0]),
         relativeFollower= np.array([]), relativeCoM= np.array([0,0,0]), mass=0, inertia= np.eye(3))

        self.absoluteFollower = np.array([[0,0,0]])
        self.absoluteOrientation = np.eye(3)

        self.absoluteOrientationDerivative = np.zeros(3)
        self.angularVelocity = np.array([0,0,0])

    def addFollower(self,follower):
        if self.relativeFollower.shape[0]==0:
            self.relativeFollower = np.array([follower])
        else:
            self.relativeFollower = np.vstack((self.relativeFollower,follower))
        self.absoluteFollower = self.relativeFollower

    def updateDerivatives(self,inputVector):
        self.absoluteOrientation_dTdq = SparseMatrix([],[],[3,3,self.dof],[1])
        self.absoluteOrientation_dTddq = SparseMatrix([],[],[3,3,self.dof,self.dof],[1,2])
        self.absoluteOrientation_dTdddq = SparseMatrix([],[],[3,3,self.dof,self.dof,self.dof],[1,2,3])

        self.relativeOrientation_dTdq = SparseMatrix([],[],[3,3,self.dof],[1])
        self.relativeOrientation_dTddq = SparseMatrix([],[],[3,3,self.dof,self.dof],[1,2])
        self.relativeOrientation_dTdddq = SparseMatrix([],[],[3,3,self.dof,self.dof,self.dof],[1,2,3])

        self.relativeAngularVelocityJacobian = SparseMatrix([],[],[3,1,self.dof],[1])
        self.H = SX.zeros(self.dof,self.dof)
        self.dH = SX.zeros(self.dof*self.dof*self.dof)
        self.ddH = SX.zeros(self.dof*self.dof*self.dof*self.dof)
        
    def update(self,inputVector):
        if self.dof==None:
            self.dof = inputVector.size()[0]
            self.updateDerivatives(inputVector)