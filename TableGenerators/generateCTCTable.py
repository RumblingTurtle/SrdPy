import numpy as np
def generateCTCTable(gcModelHandler,IKSolutionHandler, Kp, Kd,timeTable):
    count = len(timeTable)
    n = 2 * gcModelHandler.dofConfigurationSpaceRobot
    m = gcModelHandler.dofControl

    K_table = np.zeros((count,m,n))

    for i in range(count):

        t = timeTable[i]
        
        q,v,a = IKSolutionHandler.getPositionVelocityAcceleration(t)

        
        H = gcModelHandler.getJointSpaceInertiaMatrix(q)
        T = gcModelHandler.getControlMap(q)

        K_table[i] = np.linalg.pinv(T) @ H @ np.hstack(Kp, Kd)
    
    return K_table