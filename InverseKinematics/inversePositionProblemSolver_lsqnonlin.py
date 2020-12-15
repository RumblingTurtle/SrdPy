from scipy.optimize import least_squares
import numpy as np

def inversePositionProblemSolver_lsqnonlin(task, taskJacobian, value, initialGuess,opts):
    def function(q):
        res = task(q)-value
        return np.array(res).reshape(res.shape[0])

    def func_jacobian(q):
        return taskJacobian(q)

    return least_squares(function,initialGuess,func_jacobian)

