import numpy as np
from numpy import matlib

class Joint:

    def __init__(self, name, childLink, parentLink, parentFollowerNumber,
                 usedGeneralizedCoordinates,usedControlInputs,defaultJointOrientation):

        self.type = "None" # The type of the joint

        self.name = name # The name of the joint

        self.parentLink = parentLink # to which link the joint is connected
        self.childLink = childLink # which joints is connected to the joint

        self.parentFollowerNumber = parentFollowerNumber

        self.usedGeneralizedCoordinates = usedGeneralizedCoordinates
        self.usedControlInputs = usedControlInputs

        self.defaultJointOrientation = np.array(defaultJointOrientation)

        childLink.joint = self


    def update(self,inputVector):
        pass

    def actionUpdate(self,inputVector):
        pass

    def forwardKinematicsJointUpdate(self):
        self.childLink.absoluteBase = self.parentLink.absoluteFollower[self.parentFollowerNumber]
        self.childLink.absoluteOrientation = self.parentLink.absoluteOrientation@self.childLink.relativeOrientation
        rBaseToCoM = self.childLink.relativeCoM - self.childLink.relativeBase
        self.childLink.absoluteCoM = self.childLink.absoluteOrientation@rBaseToCoM+self.childLink.absoluteBase
        
        if len(self.childLink.relativeFollower)==0:
            return
        rBaseToFollower = self.childLink.relativeFollower - np.matlib.repmat(self.childLink.relativeBase,self.childLink.relativeFollower.shape[0],1)

        self.childLink.absoluteFollower = np.matlib.repmat(self.childLink.absoluteBase,self.childLink.relativeFollower.shape[0], 1) + self.childLink.absoluteOrientation.dot(rBaseToFollower.T).T

        