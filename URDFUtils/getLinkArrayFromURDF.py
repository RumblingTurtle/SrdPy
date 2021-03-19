from urdf_parser_py.urdf import URDF
import urdf_parser_py
import numpy as np
from SrdPy.LinksAndJoints import *
from SrdPy.SrdMath import rpyToRotationMatrix
import os
import meshcat.geometry as G
def getInertiaMatrixFromValues(ixx,ixy,ixz,iyy,iyz,izz):
    return np.array([[ixx,ixy,ixz],[ixy,iyy,iyz],[ixz,iyz,izz]])

def getJointClass(joint):
    if joint.name.split("_")[-1]=="floating":
        return JointFloatingBaseEuler_XYZ

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
    order = -1 
    
    
    root_link = robot.get_root()
    newLink = GroundLink()
    linkDict[root_link] = newLink
    linkArray.append(newLink)
    
    linkParserMap={}
    print("Parsing URDF:"+path)
    print("Root node: "+root_link)
    for link in robot.links:
        if link.name == robot.get_root():
            linkParserMap[link.name] = (linkDict[root_link],link)
            continue
        
        if link.inertial==None:
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
            if isinstance(link.visual.geometry,urdf_parser_py.urdf.Mesh):
                meshRelativePath = link.visual.geometry.filename
                meshPath = os.path.join(os.path.dirname(os.path.abspath(path)),meshRelativePath)
                if meshRelativePath[-3:]=="obj":
                    meshObj = G.ObjMeshGeometry.from_file(meshPath)
                elif meshRelativePath[-3:].lower() == "stl":
                    meshObj = G.StlMeshGeometry.from_file(meshPath)
                else:
                    meshObj = None
                    print("Unknown mesh format: " + meshRelativePath[-3:])

                newLink.meshObj = meshObj
            else:
                print("No mesh assigned for: "+name)
        linkDict[name] = newLink
        linkArray.append(newLink)
        linkParserMap[name] = (newLink,link)

        chainArray = robot.get_chain(robot.get_root(),link.name, joints=False, links=True, fixed=True)
        newLink.order = len(chainArray)

    coordIndex = 0
    for joint in robot.joints:
        if joint.child not in linkDict.keys() or joint.parent not in linkDict.keys():
            continue

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