class TaylorSolverHandler():
    def __init__(self,stateHandler,controllerHandler,gcModelHandler,timeHandler):
        self.stateHandler = stateHandler
        self.controllerHandler = controllerHandler
        self.gcModelHandler = gcModelHandler
        self.timeHandler = timeHandler

    def update(self):
        dt = self.timeHandler.timeLog[self.timeHandler.currentIndex + 1]\
             - self.timeHandler.timeLog[self.timeHandler.currentIndex]

        q = self.stateHandler.q
        v = self.stateHandler.v

        iH = self.gcModelHandler.getJointSpaceInertiaMatrixInverse(q)
        T = self.gcModelHandler.getControlMap(q)
        c = self.gcModelHandler.getBiasVector(q, v)

        u = self.controllerHandler.u

        a = iH @ (T@u - c)

        v = v + dt @ a
        q = q + dt @ v + 0.5 * dt**2 @ a

        self.stateHandler.q = q
        self.stateHandler.v = v
        self.stateHandler.a = a