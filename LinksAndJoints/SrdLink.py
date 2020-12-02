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