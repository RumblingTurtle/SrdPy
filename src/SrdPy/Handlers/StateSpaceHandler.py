import numpy as np
class StateSpaceHandler:
    def __init__(self,x):
        self.x = x
        self.dx = np.zeros(x.shape)
        self.dofStateSpace = x.shape[0]