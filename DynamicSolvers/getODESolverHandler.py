import numpy as np
import scipy
class ODESolverHandler():
    def __init__(self,stateHandler,controllerHandler,gcModelHandler,simulationHandler):
        self.stateHandler = stateHandler
        self.controllerHandler = controllerHandler
        self.gcModelHandler = gcModelHandler
        self.simulationHandler = simulationHandler

    def odeFunc(self, z):
        n = self.gcModelHandler.dofConfigurationSpaceRobot
        u=self.controllerHandler.us
        newQ = z[:n]
        newV = z[n:]

        H = self.gcModelHandler.getJointSpaceInertiaMatrix(newQ);
        T = self.gcModelHandler.getControlMap(newQ);
        c = self.gcModelHandler.getBiasVector(newQ, newV);

        newA = np.linalg.solve(H,T@u - c)

        return np.vstack((newV,newA))

    def update(self):
        n = self.gcModelHandler.dof_configuration_space_robot

        dt = self.simulationHandler.timeLog[self.simulationHandler.currentIndex + 1]\
             - self.simulationHandler.timeLog[self.simulationHandler.currentIndex]

        q0 = self.stateHandler.q
        v0 = self.stateHandler.v
        u = self.stateHandler.u

        ############# NOT FUNCTIONAL
        ode15s = scipy.integrate.ode(self.odeFunc)
        ode15s.set_integrator('vode', method='bdf', order=15, nsteps=3000)
        ode15s.set_initial_value(0, t0)

        #_, tape = scipy.integrate.ode(, [0, dt], [q0, v0])


        self.stateHandler.q = tape[-1, 1:n]
        self.stateHandler.v = tape[-1, n:2*n]
        self.stateHandler.a = []
        ##########

def getODESolverHandler(stateHandler,controllerHandler,gcModelHandler,simulationHandler):
    return ODESolverHandler(stateHandler,controllerHandler,gcModelHandler,simulationHandler)
