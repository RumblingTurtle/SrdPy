import numpy as np
from scipy.linalg import null_space

    
def generateConstraiedModelTable(constraintsModel, gcModelHandler, x_table,new_dimensions=None):
    count = x_table.shape[0]
    n = x_table.shape[1]
    k = constraintsModel.dofConstraint

    if new_dimensions!=None:
        nn = new_dimensions
    else:
        nn = n - 2*k

    N_table = np.zeros((count,n, nn))
    G_table = np.zeros((count,2*k, n))
    F_table = np.zeros((count,n, k))
    
    for i in range(count):
        
        x = x_table[i]
        q = np.reshape(x[:int(n/2)],(int(n/2),1))
        v = np.reshape(x[int(n/2):],(int(n/2),1))
        
        iH = gcModelHandler.getJointSpaceInertiaMatrixInverse(q)

        F = constraintsModel.getJacobian(q)
        dFdq = constraintsModel.getJacobianDerivative(q, v)
        
        Fstack = np.hstack((np.zeros((k, int(n/2))), F))
        G = np.vstack((Fstack,np.hstack((F, dFdq))))
        
        N = null_space(G)
        
        G_table[i] = G
        N_table[i] = N
        F_table[i] = np.vstack((np.zeros((int(n/2),k)),iH@F.T))

    return N_table, G_table, F_table