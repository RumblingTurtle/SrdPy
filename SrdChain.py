import numpy as np

class SrdChain:

    def __init__(self, linkArray):
        self.linkArray = linkArray
        #Sort linkArray according to Order property
        self.linkArray.sort(key=lambda link: link.order, reverse=True)
        self.jointArray = []

        generalizedCoordinates = []
        controlInputs = []
        for link in self.linkArray:
            if link.joint!=None:
                self.jointArray.append(link.joint)
                generalizedCoordinates = np.concatenate([generalizedCoordinates,link.joint.usedGeneralizedCoordinates])
                controlInputs = np.concatenate([controlInputs,link.joint.usedControlInputs])

        self.dof = len(generalizedCoordinates)
        self.controlDof = len(controlInputs)

    def update(self,q):
        for joint in self.jointArray:
            joint.update(q)