import numpy as np
import scipy
def generateConstraiedLinearModelTable(constraintsModel,A_table,B_table,c_table,x_table,dx_table,new_dimensions):
    count = A_table.shape[1]
    n = A_table.shape[0]
    m = B_table.shape[0]
    k = constraintsModel.dofConstraint

    if len(new_dimensions)!=0:
        nn = new_dimensions
    else:
        nn = n - k

    N_table = np.zeros((count,n, nn))
    G_table = np.zeros((count,2@k, n))
    An_table = np.zeros((count,nn, nn))
    Bn_table = np.zeros((count,nn, m))
    cn_table = np.zeros((count,nn))
    xn_table = np.zeros((count,nn))
    dxn_table = np.zeros((count,nn))

    for i in range(count):
        
        x = x_table[i]
        q = x[:n/2]
        v = x[n/2:]
        
        F = constraintsModel.getJacobian(q)
        dFdq = constraintsModel.getJacobianDerivative(q, v)
        
        G = np.vstack(np.hstack(np.zeros(k, n/2), F),np.hstack(F, dFdq))
        
        N =  scipy.linalg.null_space(G)
        
        G_table[i] = G
        N_table[i] = N

            
        An_table[i] = N.T @ A_table[i] @ N
        Bn_table[i] = N.T @ B_table[i]
        cn_table[i]    = N.T @ c_table[i]
        
        xn_table[i] = N.T @ x_table[i]
        dxn_table[i] = N.T @ dx_table[i]
        
    return N_table, G_table, An_table, Bn_table, cn_table, xn_table, dxn_table