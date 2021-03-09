import numpy as np
from control import lqr

def generateCLQRTable(A_table, B_table, Q_table, R_table, N_table):
    count = A_table.shape[1]
    n = A_table.shape[0]
    m = B_table.shape[0]

    K_table = np.zeros((count,m,n))

    for i in range(count):
        
        N = N_table[i]

        K, S, CLP = lqr(N.T @ A_table[i]*N, N.T @ B_table[i],N.T @ Q_table[i]@N, R_table[i] @ N.T)
        K_table[i] = K
        
    return K_table