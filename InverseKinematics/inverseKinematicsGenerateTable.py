from casadi import *
import numpy as np

from SrdPy.InverseKinematics import inversePositionProblemSolver_lsqnonlin


def inverseKinematicsGenerateTable(IKModelHandler,IKTaskHandler,initialGuess,timeTable,method=None,opts=None):
    count = len(timeTable)
    dof = IKModelHandler.dofRobot

    IKTable = np.zeros((count, dof))
    q0 = initialGuess

    for i in range(count):
        if i % np.floor(count / 100) == 0:
            print('Calculating ', str(np.floor(100 * i / count))+ '%')


        t = timeTable[i]
        taskValue = IKTaskHandler.getTask(t)
        q = inversePositionProblemSolver_lsqnonlin(IKModelHandler.getTask,IKModelHandler.getJacobian,taskValue,q0,opts).x
        IKTable[i] = q
        q0 = q
    return IKTable