import numpy as np
import numpy.matlib
from SrdPy.SparseMatrix import SparseMatrix
from casadi import *
from SrdPy.Profiling import timer

class Joint:

    def __init__(self, name, childLink, parentLink, parentFollowerNumber,
                 usedGeneralizedCoordinates,usedControlInputs,defaultJointOrientation):

        self.name = name # The name of the joint

        self.parentLink = parentLink # to which link the joint is connected
        self.childLink = childLink # which joints is connected to the joint

        self.parentFollowerNumber = parentFollowerNumber

        self.usedGeneralizedCoordinates = usedGeneralizedCoordinates
        self.usedControlInputs = usedControlInputs

        self.defaultJointOrientation = np.array(defaultJointOrientation)

        childLink.joint = self
        self.transformHandler = None
        self.sparseDerive=True

    def updateTransformDerivatives(self,inputVector):
        
        q = inputVector[self.usedGeneralizedCoordinates]
        dof = inputVector.size()[0]

        if self.childLink.dof == None:
            self.childLink.dof = dof
        if not self.sparseDerive:
            return
        relativeOrientation_dt = SparseMatrix([self.transformHandler.dT(q)],self.usedGeneralizedCoordinates,[3,3,dof],[1])
        relativeOrientation_ddt = SparseMatrix([self.transformHandler.ddT(q)],[self.usedGeneralizedCoordinates*2],[3,3,dof,dof],[1,2])
        relativeOrientation_dddt = SparseMatrix([self.transformHandler.dddT(q)],[self.usedGeneralizedCoordinates*3],[3,3,dof,dof,dof],[1,2,3])

        relativeAngularVelocityJacobian = SparseMatrix([self.transformHandler.angularVelocityJacobian()],self.usedGeneralizedCoordinates,[3,1,dof],[1])

        self.childLink.relativeAngularVelocityJacobian = relativeAngularVelocityJacobian
        p_absoluteOrientation_dt = self.parentLink.absoluteOrientation_dTdq
        p_absoluteOrientation_ddt = self.parentLink.absoluteOrientation_dTddq
        p_absoluteOrientation_dddt = self.parentLink.absoluteOrientation_dTdddq
        ###################
        #First order derivative of the transformation derivation
        #T2 = T1 * Tr2;
        #d(T2) = d(T1 * Tr2) = d(T1)* Tr2 + T1* d(Tr2)
        T1  = self.parentLink.absoluteOrientation
        dT1 = p_absoluteOrientation_dt
        Tr2  = self.transformHandler.T(q)
        dTr2 = relativeOrientation_dt

        dT2 = dT1@Tr2+T1@dTr2

        self.childLink.absoluteOrientation_dTdq = dT2
        ##################################
        #Derivation of the second order absolute transformation derivative of the child

        relativeOrientation_dt.derivative_idx = [2]
        p_absoluteOrientation_dt.derivative_idx = [1]
        d2T_mid = p_absoluteOrientation_dt@relativeOrientation_dt

        relativeOrientation_dt.derivative_idx = [1]
        p_absoluteOrientation_dt.derivative_idx = [2]

        d2T_mid = d2T_mid + p_absoluteOrientation_dt@relativeOrientation_dt
        
        d2T_sides = p_absoluteOrientation_ddt@Tr2+T1@relativeOrientation_ddt
        
        d2T = d2T_mid+d2T_sides
        d2T.derivative_idx = [1,2]
        self.childLink.absoluteOrientation_dTddq = d2T

        #####################
        #Third order derivative of the transformation
        '''
        d3T = p_absoluteOrientation_dddt@Tr2+T1@relativeOrientation_dddt

        p_absoluteOrientation_ddt.derivative_idx = [1,2]
        relativeOrientation_dt.derivative_idx = [3]
        d3T = d3T+p_absoluteOrientation_ddt@relativeOrientation_dt

        p_absoluteOrientation_dt.derivative_idx = [3]
        relativeOrientation_ddt.derivative_idx = [1,2]
        d3T =d3T+p_absoluteOrientation_dt@relativeOrientation_ddt


        p_absoluteOrientation_ddt.derivative_idx = [1,3]
        relativeOrientation_dt.derivative_idx = [2]
        d3T =d3T+p_absoluteOrientation_ddt@relativeOrientation_dt

        p_absoluteOrientation_ddt.derivative_idx = [2,3]
        relativeOrientation_dt.derivative_idx = [1]
        d3T =d3T+p_absoluteOrientation_ddt@relativeOrientation_dt

        p_absoluteOrientation_dt.derivative_idx = [1]
        relativeOrientation_ddt.derivative_idx = [2,3]
        d3T =d3T+p_absoluteOrientation_dt@relativeOrientation_ddt

        p_absoluteOrientation_dt.derivative_idx = [2]
        relativeOrientation_ddt.derivative_idx = [1,3]
        d3T =d3T+p_absoluteOrientation_dt@relativeOrientation_ddt

        d3T.derivative_idx = [1,2,3]
        self.childLink.absoluteOrientation_dTdddq = d3T

        p_absoluteOrientation_dt.derivative_idx = [1]
        p_absoluteOrientation_ddt.derivative_idx = [1,2]
        p_absoluteOrientation_dddt.derivative_idx = [1,2,3]
        '''
        currentLink = self.childLink
        linkChain = []
        while currentLink.joint!=None:
            linkChain.append(currentLink)
            currentLink = currentLink.joint.parentLink

        jacobianChains = [[],[],[]]

        offset = linkChain[0].relativeCoM-linkChain[0].relativeBase
        offset = np.reshape(offset,(3,1))
        jacobianChains[0].append(linkChain[0].absoluteOrientation_dTdq@offset)
        jacobianChains[1].append(linkChain[0].absoluteOrientation_dTddq@offset)
        #jacobianChains[2].append(linkChain[0].absoluteOrientation_dTdddq@offset)
        angularJacobian = linkChain[0].relativeAngularVelocityJacobian
        if len(linkChain)>1:
            for i,link in enumerate(linkChain[1:],start=1):
                offset = linkChain[i].relativeFollower[linkChain[i-1].parentFollowerNumber]-linkChain[i].relativeBase
                offset = np.reshape(offset,(3,1))
                jacobianChains[0].append(link.absoluteOrientation_dTdq@offset)
                jacobianChains[1].append(link.absoluteOrientation_dTddq@offset)
                #jacobianChains[2].append(link.absoluteOrientation_dTdddq@offset)
                angularJacobian = angularJacobian+link.relativeAngularVelocityJacobian
            

        angularJacobian = angularJacobian@(self.childLink.inertia@angularJacobian)

        H = SX.zeros(dof,dof)

        dH = SX.zeros(dof*dof*dof)
        #ddH = SX.zeros(dof*dof*dof*dof)

        for i in range(len(jacobianChains[0])):
            for j in range(len(jacobianChains[0])):
                
                H_1 = jacobianChains[0][i]@jacobianChains[0][j]

                dH_1 = jacobianChains[0][i]@jacobianChains[1][j]
                dH_2 = jacobianChains[1][i]@jacobianChains[0][j]
                '''
                ddH_1 = jacobianChains[2][i]@jacobianChains[0][j]
                ddH_2 = jacobianChains[0][i]@jacobianChains[2][j]
                
                jacobianChains[1][i].derivative_idx=[1,2]
                jacobianChains[1][j].derivative_idx=[2,3]
                ddH_3 = jacobianChains[1][i]@jacobianChains[1][j]

                jacobianChains[1][i].derivative_idx=[2,3]
                jacobianChains[1][j].derivative_idx=[1,2]
                ddH_4 = jacobianChains[1][i]@jacobianChains[1][j]

                jacobianChains[1][j].derivative_idx=[1,2]
                jacobianChains[1][i].derivative_idx=[1,2]
'''
                H = H + H_1*self.childLink.mass     
                dH = dH+(dH_1+dH_2)*self.childLink.mass
                #ddH = ddH+(ddH_1+ddH_2+ddH_3+ddH_4)*self.childLink.mass

        self.childLink.H = H+angularJacobian
        self.childLink.dH = dH
        #self.childLink.ddH = ddH

    def updateOrder(self):
        currentLink = self.childLink
        linkChain = []
        while currentLink.order==-1:
            linkChain.append(currentLink)
            currentLink = currentLink.joint.parentLink
        
        for link in linkChain[::-1]:
            link.order = link.joint.parentLink.order+1

    def update(self,inputVector):
        pass

    def actionUpdate(self,inputVector):
        pass

    def forwardKinematicsJointUpdate(self):
        self.childLink.absoluteBase = self.parentLink.absoluteFollower[self.childLink.parentFollowerNumber]
        self.childLink.absoluteOrientation = self.parentLink.absoluteOrientation@self.childLink.relativeOrientation
        rBaseToCoM = self.childLink.relativeCoM - self.childLink.relativeBase

        self.childLink.absoluteCoM = self.childLink.absoluteOrientation@rBaseToCoM+self.childLink.absoluteBase
        
        if len(self.childLink.relativeFollower)==0:
            return
            
        rBaseToFollower = self.childLink.relativeFollower - np.matlib.repmat(self.childLink.relativeBase,self.childLink.relativeFollower.shape[0],1)

        self.childLink.absoluteFollower = np.matlib.repmat(self.childLink.absoluteBase,self.childLink.relativeFollower.shape[0], 1) + self.childLink.absoluteOrientation.dot(rBaseToFollower.T).T

        