import numpy as np
from scipy.linalg import block_diag


def weightedPseudoinverse(A, B, alpha1, alpha2):
    '''
    Implements a weighted pseudoinverse based on Tikhonov
    regularization.
    The problem is stated as follows:
    Ax + By = z;
    find x and y, with different weights placed on them.

    it is solved as follows:
    M := [A B]; T = [I1*alpha1, 0; 0, I2*alpha2],
    where I1 and I2 are identity matrices with sizes dependant on x
    and y.

    [x; y] = (M'*M + T'*T) * M'z;
    This function returns P = pinv(M'*M + T'*T) * M' - a weighted pseudoinverse
    '''

    n1 = A.shape[1]
    n2 = B.shape[1]

    M = np.concatenate(A, B)

    T = block_diag(np.eye(n1) * alpha1, np.eye(n2) * alpha2)
    return np.linalg.pinv(M.T * M + T) * M.T