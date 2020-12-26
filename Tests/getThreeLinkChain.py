from SrdPy.LinksAndJoints import *
from SrdPy import SrdChain
import numpy as np

def getThreeLinkChain():
    groundLink = GroundLink()

    link1 = SrdLink(name="Link1", order=1,
                    inertia=np.diag([1 / 12 * 10 * 0.5, 1 / 12 * 10 * 0.5, 0.1 * 1 / 12 * 10 * 0.5]), mass=10,
                    relativeBase=[0, 0, 0], relativeFollower=[[0, 0, 0.5]], relativeCoM=[0, 0, 0.25])

    link2 = SrdLink(name="Link2", order=1,
                    inertia=np.diag([1 / 12 * 10 * 0.5, 1 / 12 * 10 * 0.5, 0.1 * 1 / 12 * 10 * 0.5]), mass=10,
                    relativeBase=[0, 0, 0], relativeFollower=[[0, 0, 0.5]], relativeCoM=[0, 0, 0.25])

    link3 = SrdLink(name="Link3", order=1,
                    inertia=np.diag([1 / 12 * 10 * 0.5, 1 / 12 * 10 * 0.5, 0.1 * 1 / 12 * 10 * 0.5]), mass=10,
                    relativeBase=[0, 0, 0], relativeFollower=[[0, 0, 0.5]], relativeCoM=[0, 0, 0.25])

    genCoordIndex = 0

    newCoordIndices = [0]
    jointGto1 = SrdJointPivotX(name="GroundToFirst", childLink=link1, parentLink=groundLink, parentFollowerNumber=0,
                               usedGeneralizedCoordinates=newCoordIndices, usedControlInputs=newCoordIndices,
                               defaultJointOrientation=np.eye(3))

    newCoordIndices = [1]
    joint1to2 = SrdJointPivotX(name="1To2", childLink=link2, parentLink=link1, parentFollowerNumber=0,
                               usedGeneralizedCoordinates=newCoordIndices, usedControlInputs=newCoordIndices,
                               defaultJointOrientation=np.eye(3))

    newCoordIndices = [2]
    joint2to3 = SrdJointPivotX(name="2To3", childLink=link3, parentLink=link2, parentFollowerNumber=0,
                               usedGeneralizedCoordinates=newCoordIndices, usedControlInputs=newCoordIndices,
                               defaultJointOrientation=np.eye(3))


    linkArray = [groundLink, link1, link2, link3]

    return SrdChain(linkArray)