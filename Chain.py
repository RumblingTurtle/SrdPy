import numpy as np

class Chain:

    def __init__(self, linkArray):
        self.linkArray = linkArray
        self.name = "Chain"
        #Sort linkArray according to Order property
        self.linkArray.sort(key=lambda link: link.order, reverse=False)
        self.jointArray = []
        generalizedCoordinates = []
        controlInputs = []
        self.links = {}
        
        for link in self.linkArray:
            if link.joint!=None:
                self.jointArray.append(link.joint)
                if len(link.joint.usedGeneralizedCoordinates) != 0:
                    generalizedCoordinates = np.concatenate([generalizedCoordinates,link.joint.usedGeneralizedCoordinates])
                    controlInputs = np.concatenate([controlInputs,link.joint.usedControlInputs])
            self.links[link.name] = link
        self.dof = len(generalizedCoordinates)
        self.controlDof = len(controlInputs)

    def get_vertex_coords(self):
        vertices = []
        for i, link in enumerate(self.linkArray):
            for followerCoord in link.absoluteFollower.tolist():
                vertices.append(link.absoluteBase)
                vertices.append(followerCoord)
        return np.array(vertices)

    def getCoM(self):
        CoM = np.zeros(3)
        mass = 0

        for link in self.linkArray:
            CoM  = CoM + link.absoluteCoM*link.mass
            mass = mass + link.mass
        
        return CoM/mass

    def __str__(self):
        out_str = self.name+"\n"
        out_str = out_str+"Links: \n"
        for i,link in enumerate(self.linkArray):
            out_str = out_str+str(i)+". "+link.name+" \n"
        return out_str
            
    def update(self,q):
        for link in self.linkArray:
            link.update(q)