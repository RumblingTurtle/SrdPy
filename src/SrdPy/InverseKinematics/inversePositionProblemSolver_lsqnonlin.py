from scipy.optimize import least_squares
import numpy as np

def inversePositionProblemSolver_lsqnonlin(task, taskJacobian, value, initialGuess):
    def function(q):
        res = task(q)-value
        return np.squeeze(np.array(res))

    def func_jacobian(q):
        return taskJacobian(q)

    return least_squares(function,initialGuess,func_jacobian,tr_solver='lsmr').x


