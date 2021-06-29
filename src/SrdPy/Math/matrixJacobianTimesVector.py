from casadi import *
def matrixJacobianTimesVector(M, var, vec):
    s1 = M.shape
    s2 = vec.shape[0]
    s3 = var.shape[0]

    P = SX.zeros(s1[0], s3)

    T = jacobian(reshape(M,s1[0]*s1[1],1), var)

    for i in range(s3):
        P[:,i] = reshape(T[:,i],s1) @ vec

    return P