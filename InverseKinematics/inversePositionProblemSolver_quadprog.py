from qpsolvers import quadprog_solve_qp
import numpy as np

def inversePositionProblemSolver_quadprog(task, taskJacobian, value, initialGuess):
    J = taskJacobian(initialGuess)
    f0 = task(initialGuess)

    H = 2*np.array(J.T@J)
    H=H+np.eye(H.shape[0])*1e-3
    f = np.squeeze(np.array(2*(f0 - J@initialGuess - value).T @ J))

    return quadprog_solve_qp(H,f,np.zeros(H.shape),np.zeros(H.shape[0]))
    