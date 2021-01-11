from SrdPy import Chain
from casadi import *

class SymbolicEngine(Chain):

    def __init__(self, linkArray):
        super(SymbolicEngine, self).__init__(linkArray)
        self.q = SX.sym("q",self.dof)
        self.v = SX.sym("v", self.dof)
        self.u = SX.sym("u", self.controlDof)

        self.update(self.q)

        self._get_joint_space_inertia_matrix = None
        self._get_bais_vector = None
        self._get_control_map = None

