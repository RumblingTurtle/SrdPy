from casadi import *
from SrdPy import SymbolicEngine


def deriveControlMap(symbolicEngine:SymbolicEngine):
    return MX.eye(symbolicEngine.dof)