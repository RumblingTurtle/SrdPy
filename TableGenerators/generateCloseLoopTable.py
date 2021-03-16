import numpy as np
def generateCloseLoopTable(A_table, B_table, c_table, K_table, x_table, u_table):
    count = A_table.shape[0]
    n = A_table.shape[1]

    AA_table = np.zeros((count,n, n))
    cc_table = np.zeros((count,n))

    for i in range(count):
        
        AA_table[i] = A_table[i] - B_table[i]@K_table[i]
        
        cc_table[i] = B_table[i] @ (K_table[i] @ x_table[i] + u_table[i]) + c_table[i]
    
    return AA_table, cc_table