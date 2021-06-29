from casadi import *

def Tensor3MatrixProduct(A,B):
    s1 = A.size()
    s2 = B.size()

    P = SX.zeros(s1[0], s2[1], s1[2]) 
    
    for i in range(1[2]):
        P[:, :, i] = A[:, :, i]@B 