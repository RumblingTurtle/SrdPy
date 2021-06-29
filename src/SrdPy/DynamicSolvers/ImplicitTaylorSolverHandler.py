from casadi import *
from scipy.optimize import least_squares

class ImplicitTaylorSolverHandler():
    def __init__(self,stateHandler,controllerHandler,gcModelHandler,timeHandler):
        self.stateHandler = stateHandler
        self.controllerHandler = controllerHandler
        self.gcModelHandler = gcModelHandler
        self.timeHandler = timeHandler

    def update(self):
        n = self.gcModelHandler.dofConfigurationSpaceRobot

        dt = self.timeHandler.timeLog[self.timeHandler.currentIndex + 1]\
             - self.timeHandler.timeLog[self.timeHandler.currentIndex]

        q0 = DM(self.stateHandler.q)
        v0 = DM(self.stateHandler.v)
        u = DM(self.controllerHandler.u)

        H0 = self.gcModelHandler.getJointSpaceInertiaMatrixInverse(q0)
        T0 = self.gcModelHandler.getControlMap(q0)
        c0 = self.gcModelHandler.getBiasVector(q0, v0)

        a0 = DM(pinv(H0) @ (T0@u - c0))

        def objective(z):
            n = self.gcModelHandler.dofConfigurationSpaceRobot
            newQ = z[:n]
            newV = z[n:2*n]
            newA = z[2*n:]

            H = self.gcModelHandler.getJointSpaceInertiaMatrixInverse(newQ)
            T = self.gcModelHandler.getControlMap(newQ)
            c = self.gcModelHandler.getBiasVector(newQ, newV)

            comp1 = H@newA + c - T@u
            comp2 = v0 + dt * newA - newV
            comp3 = q0 + dt * newV + 0.5 * dt**2 * newA - newQ

            return np.squeeze(np.vstack((comp1,comp2,comp3)))

        stacked = np.squeeze(np.vstack((q0, v0, a0)))
        z = least_squares(objective,stacked).x

        self.stateHandler.q = z[:n]
        self.stateHandler.v = z[n:2*n]
        self.stateHandler.a = z[2*n:]