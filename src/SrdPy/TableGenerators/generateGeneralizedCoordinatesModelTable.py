from casadi import *
import numpy as np

def generateGeneralizedCoordinatesModelTable(gcModelHandler,IKSolutionHandler,timeTable):
    count = len(timeTable)
    n = 2 * gcModelHandler.dofConfigurationSpaceRobot
    m = gcModelHandler.dofControl
    
    H_table = np.zeros((count,n, n))
    iH_table = np.zeros((count,n, n)) 
    T_table = np.zeros((count,n, m)) 
    c_table = np.zeros((count,n)) 
    q_table = np.zeros((count,n)) 
    v_table = np.zeros((count,n)) 
    a_table = np.zeros((count,n)) 
    u_table = np.zeros((count,m)) 

    for i in range(count):
        
        t = timeTable[i]
        
        q,v,a = IKSolutionHandler.getPositionVelocityAcceleration(t)

        H = gcModelHandler.getJointSpaceInertiaMatrix(q)
        iH = gcModelHandler.getJointSpaceInertiaMatrixInverse(q)
        T = gcModelHandler.getControlMap(q)
        c = gcModelHandler.getBiasVector(q, v)
        
        u = np.linalg.pinv(T)@(H@a + c)
        
        H_table[i] = H
        iH_table[i] = iH
        T_table[i]= T
        c_table[i] = c
        
            
        q_table[i] = q
        v_table[i] = v
        a_table[i] = a
        u_table[i] = u
    
    return H_table, iH_table, T_table, c_table, q_table, v_table, a_table, u_table