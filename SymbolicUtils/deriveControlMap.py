from casadi import *
from SrdPy import SymbolicEngine


def deriveControlMap(symbolicEngine:SymbolicEngine):
    return SX.eye(symbolicEngine.dof)