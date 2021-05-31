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
        self.interpolator = interpolate.interp1d(x=self.timeTable,y=self.IKTable,axis=0,kind=self.method,bounds_error=False,fill_value=(self.IKTable[0],self.IKTable[-1]))

    def getPosition(self,t):
        return self.interpolator(t)


    def getPositionVelocityAcceleration(self,t):
        q = self.getPosition(t).T

        task_v = self.IKTaskHandler.getDerivative(t)
        task_a = self.IKTaskHandler.getTaskSecondDerivative(t)

        J = np.array(self.IKModelHandler.getJacobian(q))
        v = np.linalg.pinv(J)@task_v

        dJ = np.array(self.IKModelHandler.getJacobianDerivative(q, v))
      
        jVCat = np.vstack([np.hstack([J,  np.zeros(J.shape)]),np.hstack([dJ,J])])
        jVCat = np.linalg.pinv(jVCat)
        taskCat = np.vstack((task_v, task_a))

        res = jVCat @ taskCat

        v = res[:self.IKModelHandler.dofRobot]
        a = res[self.IKModelHandler.dofRobot:]

        return q, np.squeeze(v), np.squeeze(a)
