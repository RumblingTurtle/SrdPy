import numpy as np
from casadi import *
class Suit:
    def __init__(self) -> None:
        self.M = None
        self.U = None
        self.S = None
        self.V = None
        self.tol = None
        self.s = None
        self.rank = None
        self.pinv = None
        self.orth = None 
        self.left_null = None


def svd_suit(M,tol=None):
    suit = Suit()

    suit.M = M

    U, sdiag, VH = numpy.linalg.svd(DM(M))
    S = np.zeros(M.shape)
    np.fill_diagonal(S, sdiag)
    V = VH.T.conj()
    suit.U = U
    suit.S = S
    suit.V = V

    if min(S.shape) > 1:
        s = np.diag(S)
    else:
        s = S[0]
    
    suit.s = s

    #matlab-style automatic tolerance
    if tol == None:
        suit.tol = max(M.shape) * np.spacing(np.linalg.norm(s, np.inf))
    else:
        suit.tol = tol
    

    #rank
    suit.rank = np.sum(s > suit.tol)

    #pinv
    V = V[:,:suit.rank]
    U = U[:,:suit.rank]
    s = s[:suit.rank]
    s = np.power(np.ravel(s),-1)
    suit.pinv = (V*s.T)@U.T

    #null
    V = suit.V
    suit.null = V[:,suit.rank:]

    #row_space
    suit.row_space = V[:,:suit.rank]

    #orth (column space)
    U = suit.U
    U = U[:, :suit.rank]
    suit.orth = U

    #left_null
    U = suit.U
    U = U[:, suit.rank:]
    suit.left_null = U
    return suit