from scipy import interpolate
import numpy as np
from casadi import *
class IKSolutionHandler():
    def __init__(self,IKModelHandler,IKTaskHandler,timeTable,IKTable,method):
        self.IKModelHandler = IKModelHandler
        self.IKTaskHandler = IKTaskHandler
        self.timeTable = timeTable
        self.IKTable = IKTable
        self.method = method

    def getPosition(self,t):
        a = interpolate.interp1d(x=self.timeTable,y=self.IKTable[:,0],kind=self.method)(t)
        b = interpolate.interp1d(x=self.timeTable, y=self.IKTable[:, 1], kind=self.method)(t)
        c = interpolate.interp1d(x=self.timeTable, y=self.IKTable[:, 2], kind=self.method)(t)
        return a,b,c


    def getPositionVelocityAcceleration(self,t):
        q = self.getPosition(t)

        task_v = self.IKTaskHandler.getDerivative(t)
        task_a = self.IKTaskHandler.getTaskSecondDerivative(t)

        J = self.IKModelHandler.getJacobian(q)
        v = np.linalg.pinv(J)@task_v

        dJ = self.IKModelHandler.getJacobianDerivative(q, v)
        
        jHCat = horzcat(J,  SX.zeros((J.shape)))
        jVCat = vertcat(jHCat,horzcat(dJ,J))
        taskCat = np.vstack((task_v, task_a))

        res = pinv(jVCat) @ taskCat

        v = res[:self.IKModelHandler.dofRobot]
        a = res[self.IKModelHandler.dofRobot:]

        return [q, v, a]

def getIKSolutionHandler(IKModelHandler,IKTaskHandler,timeTable,IKTable,method="linear"):
    return IKSolutionHandler(IKModelHandler,IKTaskHandler,timeTable,IKTable,method)

