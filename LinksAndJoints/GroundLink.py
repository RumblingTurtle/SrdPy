import numpy as np
from SrdPy.LinksAndJoints.SrdLink import SrdLink

class GroundLink(SrdLink):
    def __init__(self):
        self.order = 0
        self.name = 'Ground'
        self.relativeBase = np.array([0,0,0])
        self.relativeFollower= np.array([[0,0,0]])
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

    def addFollower(self,follower):
        pass

    def update(self,q):
        pass