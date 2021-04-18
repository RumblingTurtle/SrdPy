from SrdPy import Chain
from casadi import *

class SymbolicEngine(Chain):

    def __init__(self, linkArray):
        super(SymbolicEngine, self).__init__(linkArray)
        self.q = SX.sym("q",self.dof)
        self.v = SX.sym("v", self.dof)
        self.u = SX.sym("u", self.controlDof)

        self.update(self.q)

    def getH(self):
        result = SX.zeros(self.dof,self.dof)
        for link in self.linkArray:
            result = result+link.H
        return result
        
    def get_dH(self):
        result = SX.zeros(self.dof*self.dof*self.dof)
        for link in self.linkArray:
            result = result+link.dH
        return result

    def get_ddH(self):
        result = SX.zeros(self.dof*self.dof*self.dof*self.dof)
        for link in self.linkArray:
            result = result+link.ddH
        return result
