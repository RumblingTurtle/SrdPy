import numpy as np

class NoInputController():
    def __init__(self,gcModelHandler):
        self.u = []
        self.gcModelHandler = gcModelHandler

    def update(self):
        self.u = np.zeros((self.gcModelHandler.dofControl,1))