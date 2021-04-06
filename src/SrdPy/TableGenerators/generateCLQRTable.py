import numpy as np
from control import lqr

def generateCLQRTable(A_table, B_table, Q_table, R_table, N_table):
    count = A_table.shape[0]
    n = A_table.shape[2]
    m = B_table.shape[2]

    K_table = np.zeros((count,m,n))

    for i in range(count):
        
        N = N_table[i]

        a = N.T @ A_table[i]@N
        b = N.T @ B_table[i]
        q = N.T @ Q_table[i]@N
        r = R_table[i]

        K, S, CLP = lqr(a, b, q, r)
        K_table[i] = K@ N.T
        
    return K_table