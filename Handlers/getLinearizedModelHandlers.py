from casadi import *

class LinearizedModelHandlers:

    def __init__(self,description):
        so_path = description["path"] + "/" + description["casadi_cCodeFilename"] + ".so"

        self.dofConfigurationSpaceRobot = description["dofConfigurationSpaceRobot"]
        self.dofStateSpaceRobot = description["dofStateSpaceRobot"]
        self.dofControl = description["dofControl"]
        self.Ahandler = external(description["functionName_A"], so_path)
        self.Bhandler = external(description["functionName_B"], so_path)
        #self.Chandler = external(description["functionName_c"], so_path)

    def getA(self,q, v, u, iH):
        return self.Ahandler(DM(q),DM(v),DM(u),DM(iH))

    def getB(self,q, u, iH):
        return self.Bhandler(DM(q),DM(u),DM(iH))

    def getC(self,**args):
        return self.Chandler(args)

def getLinearizedModelHandlers(description):
    return LinearizedModelHandlers(description)