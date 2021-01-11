from urdf_parser_py.urdf import URDF
import numpy as np
from SrdPy.LinksAndJoints import *
from SrdPy.SrdMath import rpyToRotationMatrix
import os
import meshcat.geometry as G
def getInertiaMatrixFromValues(ixx,ixy,ixz,iyy,iyz,izz):
    return np.array([[ixx,ixy,ixz],[ixy,iyy,iyz],[ixz,iyz,izz]])

def getJointClass(joint):

   if joint.type == "revolute":
       if joint.axis[0]>0:
           return JointPivotX
       if joint.axis[1] > 0:
           return JointPivotY
       if joint.axis[2] > 0:
           return JointPivotZ

   if joint.type == "fixed":
        return JointFixed




def getLinkArrayFromURDF(path,parseMeshses=False):
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


        newLink = Link(name=name,order=order,
                        inertia=inertiaMatrix, mass=mass,
                        relativeBase=[0, 0, 0], relativeFollower=[], relativeCoM=relativeCOM)
        if parseMeshses and link.visual!=None:
            meshRelativePath = link.visual.geometry.filename
            meshPath = os.path.join(os.path.dirname(os.path.abspath(path)),meshRelativePath)
            if meshRelativePath[-3:]=="obj":
                meshObj = G.ObjMeshGeometry.from_file(meshPath)
            elif meshRelativePath[-3:] == "stl":
                meshObj = G.StlMeshGeometry.from_file(meshPath)
            else:
                meshObj = None
                print("Unknown mesh format: " + meshRelativePath[-3:])

            newLink.meshObj = meshObj
        order = order+1
        linkDict[name] = newLink
        linkArray.append(newLink)

    coordIndex = 0
    for joint in robot.joints:
        child = linkDict[joint.child]
        parent = linkDict[joint.parent]
        defaultOrientation = rpyToRotationMatrix(joint.origin.rpy)

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