from casadi import *
from SrdPy import SymbolicEngine


def deriveControlMap(symbolicEngine:SymbolicEngine):
    return DM.eye(symbolicEngine.dof)