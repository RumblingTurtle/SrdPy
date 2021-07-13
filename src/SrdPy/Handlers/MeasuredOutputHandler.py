import numpy as np
class MeasuredOutputHandler:
    def __init__(self,stateSpaceHandler,C):
        self.y = None
        self.stateSpaceHandler = stateSpaceHandler
        self.C = C

        self.dofMeasuredOutput = len(self.C)

    def update(self):
        self.y = np.array(self.C)@self.stateSpaceHandler.x