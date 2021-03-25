from casadi import *
from SrdPy import SymbolicEngine
from scipy.linalg import block_diag

def deriveControlMapFloating(symbolicEngine:SymbolicEngine):
    return block_diag(DM.zeros(6,6),DM.eye(symbolicEngine.dof-6))