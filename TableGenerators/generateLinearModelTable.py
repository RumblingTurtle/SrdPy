from casadi import *
import numpy as np

def generateLinearModelTable(gcModelHandler,linearizedModelHandler,IKSolutionHandler, timeTable):
    count = len(timeTable)
    n = 2 * gcModelHandler.dofConfigurationSpaceRobot
    m = gcModelHandler.dofControl

    A_table = np.zeros((count,n, n))
    B_table = np.zeros((count,n, m))
    c_table = np.zeros((count,n))
    x_table = np.zeros((count,n))
    u_table = np.zeros((count,m))
    dx_table = np.zeros((count,n))

    for i in range(count):
        
        t = timeTable[i]
        
        q,v,a = IKSolutionHandler.getPositionVelocityAcceleration(t)
        
        H = gcModelHandler.getJointSpaceInertiaMatrix(q)
        iH = gcModelHandler.getJointSpaceInertiaMatrixInverse(q)
        T = gcModelHandler.getControlMap(q)
        c = gcModelHandler.getBiasVector(q, v)
        
        u = np.linalg.pinv(T)@(H@a + c)
        
        A_table[i]= linearizedModelHandler.getA(q, v, u, iH)
        B_table[i]= linearizedModelHandler.getB(q, u, iH)
            
        x_table[i] = np.hstack((q, v))
        u_table[i] = np.squeeze(u)
        
        dx_table[i] = np.hstack((v, a))
        
        c_table[i] = dx_table[i] - A_table[i]@x_table[i] - B_table[i]@u_table[i]

    return A_table, B_table, c_table, x_table, u_table, dx_table