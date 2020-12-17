class TaylorSolverHandler():
    def __init__(self,stateHandler,controllerHandler,gcModelHandler,simulationHandler):
        self.stateHandler = stateHandler
        self.controllerHandler = controllerHandler
        self.gcModelHandler = gcModelHandler
        self.simulationHandler = simulationHandler

    def update(self):
        dt = self.simulationHandler.timeLog[self.simulationHandler.currentIndex + 1]\
             - self.simulationHandler.timeLog[self.simulationHandler.currentIndex]

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

def getTaylorSolverHandler(stateHandler,controllerHandler,gcModelHandler,simulationHandler):
    return TaylorSolverHandler(stateHandler,controllerHandler,gcModelHandler,simulationHandler)
