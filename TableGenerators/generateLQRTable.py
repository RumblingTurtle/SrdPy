import numpy as np
from control import lqr

def generateLQRTable(A_table, B_table, Q_table, R_table):
    count = A_table.shape[1]
    n = A_table.shape[0]
    m = B_table.shape[0]

    K_table = np.zeros((count,m,n))

    for i in range(count):
        K, S, CLP =  lqr(A_table[i], B_table[i], Q_table[i], R_table[i])
        K_table[i] = K
        
    return K_table