from SrdPy import SrdChain
from casadi import *

class SymbolicEngine(SrdChain):

    def __init__(self, linkArray):
        super(SymbolicEngine, self).__init__(linkArray)
        self.q = MX.sym("q",self.dof)
        self.v = MX.sym("v", self.dof)
        self.u = MX.sym("u", self.controlDof)

        #self.update(self.q)