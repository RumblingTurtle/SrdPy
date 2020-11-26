import numpy as np
from numpy import matlib
from SRD import SrdMath
import numpy as np

class SrdLink:
    def __init__(self, name, order, relativeBase, relativeFollower, relativeCoM, mass, inertia):

        self.name = name #The name of the link
        self.joint = None #function handle, to be assigned

    ##############################################
    #geometrical properties
        self.relativeBase = np.array(relativeBase)         #Position of the Base in the local RF

        self.relativeFollower = np.array(relativeFollower)      #Position of the Follower in the local RF
        self.relativeCoM = np.array(relativeCoM)          #Position of the Center of Mass in the local RF
        self.relativeOrientation = np.eye(3)   #Orientation of the local RF relative to the FR of the parent link
                                    #there can be more than one follower, in that case
                                    #each gets a column in RelativeFollower matrix

        self.absoluteBase = np.array([])         #Position of the Base in the ground RF
        self.absoluteCoM = np.array([])         #Position of the Center of Mass in the ground RF
        self.absoluteOrientation = np.array([])   #Orientation of the local RF relative to the ground RF
        self.absoluteFollower = np.array([])      #Position of the Follower in the ground RF
                                    #(also see description of RelativeFollower)

        ##############################################
        #inertial properties
        self.mass = mass               #the mass of the link
        self.inertia = np.array(inertia)             #the inertia of the link


        ##############################################
        #Settings
        self.useAbsoluteCoordinates = False

        ##############################################
        #visual properties
        self.color = []
        self.stlPath = ""
        self.mesh = {'Faces': [], 'Vertices': []}

        self.toDisplay = []

        #if true the Update function will not change the .AbsoluteOrientation
        #property
        #if false .AbsoluteOrientation = ParentLink.AbsoluteOrientation * .RelativeOrientation

        self.parentLink = None
        self.parentFollowerNumber = 0

        self.order = 0              #order of the link, used to decide the order of link updates
                                    #if order == 0 then it defines the link as ground,
                                    #if order > 0 - as a normal link; ground
                                    #has no parent link

        ###################
        # for symbolic/AD derivations
        self.jacobianCenterOfMass = []
        self.jacobianAngularVelocity = []

        self.absoluteOrientation_derivative = []
        self.angularVelocity = []
        ###################

        #dynamically added additional parameters
        self.calculated = []

class GroundLink(SrdLink):
    def __init__(self):
        self.order = 0
        self.name = 'Ground'
        self.relativeBase = np.array([0,0,0])
        self.relativeFollower= np.array([[0,0,0]])
        self.relativeCoM = np.array([0,0,0])
        self.relativeOrientation = np.eye(3)

        self.absoluteFollower = np.array([[0,0,0]])

        self.absoluteOrientation = np.eye(3)

        self.mass = 0
        self.inertia = np.eye(3)


class SrdJoint:

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

        parentLink.joint = self


    def update(self,inputVector):
        pass

    def actionUpdate(self,inputVector):
        pass

    def forwardKinematicsJointUpdate(self):
        self.childLink.absoluteBase = self.parentLink.absoluteFollower[self.parentFollowerNumber]
        self.childLink.absoluteOrientation = self.parentLink.absoluteOrientation * self.childLink.relativeOrientation

        rBaseToFollower = self.childLink.relativeFollower - np.matlib.repmat(self.childLink.relativeBase,self.childLink.relativeFollower.shape[0],1)
        rBaseToCoM = self.childLink.relativeCoM - self.childLink.relativeBase

        self.childLink.absoluteFollower = np.matlib.repmat(self.childLink.absoluteBase,self.childLink.relativeFollower.shape[0], 1) + self.childLink.absoluteOrientation.dot(rBaseToFollower.T).T

        self.childLink.absoluteCoM = self.childLink.absoluteBase + self.childLink.absoluteOrientation * rBaseToCoM

class SrdJointFixed(SrdJoint):
    def __init__(self, name, childLink, parentLink, parentFollowerNumber,
                 usedGeneralizedCoordinates,usedControlInputs,defaultJointOrientation):
        self.type = "Fixed"
        super(SrdJointFixed, self).__init__(name, childLink, parentLink, parentFollowerNumber,
                                            usedGeneralizedCoordinates,usedControlInputs,defaultJointOrientation)
    @staticmethod
    def getJointInputsRequirements():
        return 0

    def update(self, inputVector):
        self.relativeOrientation = self.defaultJointOrientation
        self.forwardKinematicsJointUpdate()

class SrdJointPivotX(SrdJoint):
    def __init__(self, name, childLink, parentLink, parentFollowerNumber,
                 usedGeneralizedCoordinates,usedControlInputs,defaultJointOrientation):
        self.type = "PivotX"
        super(SrdJointPivotX, self).__init__(name, childLink, parentLink, parentFollowerNumber,
                                            usedGeneralizedCoordinates,usedControlInputs,defaultJointOrientation)
    @staticmethod
    def getJointInputsRequirements():
        return 1

    def update(self,inputVector):
        q = inputVector[self.usedGeneralizedCoordinates]

        self.relativeOrientation = self.defaultJointOrientation * SrdMath.rotationMatrix3Dx(q)

        self.forwardKinematicsJointUpdate()

    def actionUpdate(self,inputVector):
        u = inputVector[self.usedControlInputs]

        Child_T = self.childLink.absoluteOrientation

        torque_child  = Child_T * [u, 0, 0]
        torque_parent = Child_T * [-u, 0, 0]

        child_Jw = self.childLink.jacobianAngularVelocity
        parent_Jw = self.parentLink.jacobianAngularVelocity

        gen_force_child = child_Jw.T  * torque_child
        gen_force_parent = parent_Jw.T * torque_parent

        return gen_force_child + gen_force_parent

class SrdJointPivotY(SrdJoint):
    def __init__(self, name, childLink, parentLink, parentFollowerNumber,
                 usedGeneralizedCoordinates, usedControlInputs, defaultJointOrientation):
        self.type = "PivotY"
        super(SrdJointPivotY, self).__init__(name, childLink, parentLink, parentFollowerNumber,
                                             usedGeneralizedCoordinates, usedControlInputs, defaultJointOrientation)
    @staticmethod
    def getJointInputsRequirements():
        return 1

    def update(self, inputVector):
        q = inputVector[self.usedGeneralizedCoordinates]

        self.relativeOrientation = self.defaultJointOrientation * SrdMath.rotationMatrix3Dy(q)

        self.forwardKinematicsJointUpdate()

    def actionUpdate(self,inputVector):
        u = inputVector[self.usedControlInputs]

        Child_T = self.childLink.absoluteOrientation

        torque_child  = Child_T * [0, u, 0]
        torque_parent = Child_T * [0, -u, 0]

        child_Jw = self.childLink.jacobianAngularVelocity
        parent_Jw = self.parentLink.jacobianAngularVelocity

        gen_force_child = child_Jw.T  * torque_child
        gen_force_parent = parent_Jw.T * torque_parent

        return gen_force_child + gen_force_parent

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

        self.relativeOrientation = self.defaultJointOrientation * SrdMath.rotationMatrix3Dz(q)

        self.forwardKinematicsJointUpdate()

    def actionUpdate(self,inputVector):
        u = inputVector[self.usedControlInputs]

        Child_T = self.childLink.absoluteOrientation

        torque_child  = Child_T * [0, 0, u]
        torque_parent = Child_T * [0, 0, -u]

        child_Jw = self.childLink.jacobianAngularVelocity
        parent_Jw = self.parentLink.jacobianAngularVelocity

        gen_force_child = child_Jw.T  * torque_child
        gen_force_parent = parent_Jw.T * torque_parent

        return gen_force_child + gen_force_parent
