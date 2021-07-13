import numpy as np
from .LTI_CLQE import LTI_CLQE
from .LTI_CLQE import LTI_System

class CLQE:
    def __init__(self,timeHandler, controlInputStateSpaceHandler, inverseDynamicsHandler,gcModel,
    linearizedModel,constraintsModel,C,controllerSettings,observerSettings,tol):
        self.solution = None

        self.timeHandler = timeHandler
        self.controlInputStateSpaceHandler = controlInputStateSpaceHandler
        self.inverseDynamicsHandler = inverseDynamicsHandler
        self.linearizedModel = linearizedModel
        self.gcModel=gcModel
        self.constraintsModel = constraintsModel
        self.C = C
        self.controllerSettings = controllerSettings
        self.observerSettings = observerSettings
        self.tol = tol
    
    def update(self):
        t = self.timeHandler.currentTime
        
        dof = self.gcModel.dofConfigurationSpaceRobot
        k = self.constraintsModel.dofConstraint
        
        desired = self.controlInputStateSpaceHandler.getX_dx(t)
        desired_x =  desired[0]
        desired_dx = desired[1]
        desired_q = desired_x[:dof]
        desired_v = desired_x[dof:]
        
        desired_u = self.inverseDynamicsHandler.u
        
        H = self.gcModel.getJointSpaceInertiaMatrix(desired_q)
        c = self.gcModel.getBiasVector(desired_q, desired_v)
        T = self.gcModel.getControlMap(desired_q)
        n = self.gcModel.dofConfigurationSpaceRobot
                    
        F  = self.constraintsModel.getJacobian(desired_q)
        dF = self.constraintsModel.getJacobianDerivative(desired_q, desired_v)
        G = np.vstack((np.hstack([np.zeros((k, dof)), F]) ,np.hstack([F, dF])))
        k = self.constraintsModel.dofConstraint
        
        M = np.vstack([np.hstack((H, -F.T)),
                np.hstack([F, np.zeros((k, k))])])
        iM = np.linalg.pinv(M)
        Ma = iM[:n,:]
        
        a0 = Ma@np.vstack((T@desired_u - c, -dF@desired_v))
        g = np.vstack((desired_v,a0))
        
        A = self.linearizedModel.getA()
        B = self.linearizedModel.getB()
        
        System = LTI_System(A=A,B=B,C=np.array(self.C),G=G,g=g,tol=self.tol,controllerSettings=self.controllerSettings,
        observerSettings=self.observerSettings,x_desired=desired_x,dx_desired=desired_dx)

        saveComputation = 1
        self.solution = LTI_CLQE(System, saveComputation)
    