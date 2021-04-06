from casadi import *

class LinearizedModelHandler:

    def __init__(self,description):
        so_path = description["path"] + "/" + description["casadi_cCodeFilename"] + ".so"

        self.dofConfigurationSpaceRobot = description["dofConfigurationSpaceRobot"]
        self.dofStateSpaceRobot = description["dofStateSpaceRobot"]
        self.dofControl = description["dofControl"]

        if description["useJIT"]:
            imp = Importer(description["path"] + "/" +description["casadi_cCodeFilename"]+".c","clang")
        else:
            imp = so_path
            
        self.Ahandler = external(description["functionName_A"], imp)
        self.Bhandler = external(description["functionName_B"], imp)
        #self.Chandler = external(description["functionName_c"], imp)

    def getA(self,q, v, u, iH):
        return self.Ahandler(DM(q),DM(v),DM(u),DM(iH))

    def getB(self,q, u, iH):
        return self.Bhandler(DM(q),DM(u),DM(iH))

    def getC(self,**args):
        return self.Chandler(args)