import numpy as np
from SrdPy.LinksAndJoints import Link

class GroundLink(Link):
    def __init__(self):
        self.order = 0
        self.name = 'Ground'
        self.relativeBase = np.array([0,0,0])
        self.relativeFollower= np.array([])
        self.relativeCoM = np.array([0,0,0])
        self.absoluteCoM = np.array([0, 0, 0])
        self.mass = 0
        self.relativeOrientation = np.eye(3)
        self.absoluteBase = np.array([0,0,0])
        self.absoluteFollower = np.array([[0,0,0]])
        self.joint = None
        self.absoluteOrientation = np.eye(3)

        self.mass = 0
        self.inertia = np.eye(3)


        self.absoluteOrientationDerivative = np.zeros(3)
        self.angularVelocity = np.array([0,0,0])
        self.meshObj = None
        self.primitiveObj = None

    def addFollower(self,follower):
        if self.relativeFollower.shape[0]==0:
            self.relativeFollower = np.array([follower])
        else:
            self.relativeFollower = np.vstack((self.relativeFollower,follower))
        self.absoluteFollower = self.relativeFollower

    def update(self,q):
        pass