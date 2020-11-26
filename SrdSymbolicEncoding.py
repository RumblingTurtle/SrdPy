from SrdChain import SrdChain
from casadi import *

class SrdSymbolicEncoding(SrdChain):

    def __init__(self, linkArray):
        super(linkArray)
        self.q = MX.sym("q",self.dof,1)
        self.v = MX.sym("v", self.dof, 1)
        self.u = MX.sym("u", self.controlDof, 1)

        self.update(self.q)