from urdf_parser_py.urdf import URDF
import numpy as np
from SrdPy.LinksAndJoints import *

def getInertiaMatrixFromValues(ixx,ixy,ixz,iyy,iyz,izz):
    return np.array([[ixx,ixy,ixz],[ixy,iyy,iyz],[ixz,iyz,izz]])

def getJointClass(joint):
   if joint.type == "revolute":
       if joint.origin.xyz[0]>0:
           return SrdJointPivotX
       if joint.origin.xyz[1] > 0:
           return SrdJointPivotY
       if joint.origin.xyz[2] > 0:
           return SrdJointPivotZ

   if joint.type == "fixed":
        return SrdJointFixed




def getLinkArrayFromUrdf(path,parseMeshses=False):
    robot = URDF.from_xml_file(path)
    linkArray = []
    linkDict = {}
    jointArray = []
    order = 0
    for link in robot.links:

        if link.inertial==None:
            newLink = GroundLink()
            linkDict[link.name] = newLink
            linkArray.append(newLink)
            order=order+1
            continue
            
        name = link.name
        inertia = link.inertial.inertia

        inertiaMatrix = getInertiaMatrixFromValues(inertia.ixx,inertia.ixy,inertia.ixz,
                                                   inertia.iyy,inertia.iyz,inertia.izz)
        relativeCOM = link.inertial.origin.xyz
        mass = link.inertial.mass

        newLink = SrdLink(name=name,order=order,
                        inertia=inertiaMatrix, mass=mass,
                        relativeBase=[0, 0, 0], relativeFollower=[], relativeCoM=relativeCOM)
        order = order+1
        linkDict[name] = newLink
        linkArray.append(newLink)

    coordIndex = 0
    for joint in robot.joints:
        child = linkDict[joint.child]
        parent = linkDict[joint.parent]
        if not np.any(joint.origin.rpy):
            defaultOrientation = np.eye(3)
        else:
            defaultOrientation = np.diag(joint.origin.rpy)

        parentFollower = joint.origin.xyz

        jointClass = getJointClass(joint)
        jointUsedCoords = jointClass.getJointInputsRequirements()
        coordIndices = np.arange(coordIndex,coordIndex+jointUsedCoords)

        parent.addFollower(parentFollower)
        parentFollowerIndex = len(parent.relativeFollower)-1
        newJoint = jointClass(name=joint.name, childLink=child, parentLink=parent, parentFollowerNumber=parentFollowerIndex,
                                   usedGeneralizedCoordinates=coordIndices, usedControlInputs=coordIndices,
                                   defaultJointOrientation=defaultOrientation)
        coordIndex=coordIndex+jointUsedCoords
        jointArray.append(newJoint)
    return linkArray