from scipy import interpolate
import numpy as np
from casadi import *
class IKSolutionHandler():
    def __init__(self,IKModelHandler,IKTaskHandler,timeTable,IKTable,method="linear"):
        self.IKModelHandler = IKModelHandler
        self.IKTaskHandler = IKTaskHandler
        self.timeTable = timeTable
        self.IKTable = IKTable
        self.method = method
        self.dofRobot = self.IKModelHandler.dofRobot
        self.timeStart = max(self.IKModelHandler.timeStart,self.IKTaskHandler.timeStart)
        self.timeExpiration = min(self.IKModelHandler.timeExpiration,self.IKTaskHandler.timeExpiration)

    def getPosition(self,t):
        result = []
        for i in range(self.IKTable.shape[1]):
            a = interpolate.interp1d(x=self.timeTable,y=self.IKTable[:,i],kind=self.method)(t)
            result.append(a)
        return np.array(result)


    def getPositionVelocityAcceleration(self,t):
        q = self.getPosition(t).T

        task_v = self.IKTaskHandler.getDerivative(t)
        task_a = self.IKTaskHandler.getTaskSecondDerivative(t)

        J = np.array(self.IKModelHandler.getJacobian(q))
        v = np.linalg.pinv(J)@task_v

        dJ = np.array(self.IKModelHandler.getJacobianDerivative(q, v))
        
        jHCat = np.hstack([J,  np.zeros(J.shape)])
        jVCat = np.vstack([jHCat,np.hstack([dJ,J])])
        jVCat = np.linalg.pinv(np.array(jVCat))
        taskCat = np.vstack((task_v, task_a))

        res = np.array(jVCat @ taskCat)

        res = np.reshape(res,(2*self.IKModelHandler.dofRobot))
        v = res[:self.IKModelHandler.dofRobot]
        a = res[self.IKModelHandler.dofRobot:]

        return q, v, a
