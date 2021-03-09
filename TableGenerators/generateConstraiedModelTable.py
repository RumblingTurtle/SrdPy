import numpy as np
import scipy
def generateConstraiedModelTable(constraintsModel, x_table,new_dimensions):
    count = x_table.shape[0]
    n = x_table.shape[1]
    k = constraintsModel.dofConstraint

    if len(new_dimensions)!=0:
        nn = new_dimensions
    else:
        nn = n - k

    N_table = np.zeros((count,n, nn))
    G_table = np.zeros((count,2*k, n))

    for i in range(count):
        
        x = x_table[i]
        q = np.reshape(x[:int(n/2)],(int(n/2),1))
        v = np.reshape(x[int(n/2):],(int(n/2),1))
        
        F = constraintsModel.getJacobian(q)
        dFdq = constraintsModel.getJacobianDerivative(q, v)
        
        Fstack = np.hstack((np.zeros((k, int(n/2))), F))

        G = np.vstack((Fstack,np.hstack((F, dFdq))))
        
        N =  scipy.linalg.null_space(G)
        
        G_table[i] = G
        N_table[i] = N

    return N_table, G_table