import numpy as np
from scipy.linalg import block_diag
from scipy.spatial import ConvexHull
from itertools import product,chain
from multiprocessing import Pool
from casadi import *

numberOfWorkers = 8
simplifyFunctions = True
useParallel = True
simplifySteps = 1


def rotationMatrix2D(q):
    return [[np.cos(q), -np.sin(q)],
            [np.sin(q), np.cos(q)]]


def rotationMatrix3Dx(q):
    return [[1, 0, 0],
            [0, np.cos(q), -np.sin(q)],
            [0, np.sin(q), np.cos(q)]]


def rotationMatrix3Dy(q):
    return [[np.cos(q), 0, np.sin(q)],
            [0, 1, 0],
            [-np.sin(q), 0, np.cos(q)]]


def rotationMatrix3Dz(q):
    return [[np.cos(q), -np.sin(q), 0],
            [np.sin(q), np.cos(q), 0],
            [0, 0, 1]]


def crossProductMatrix2D(a):
    return [-a[1], a[0]]


def crossProductMatrix3D(a):
    return [[0, -a[2], a[1]],
            [a[2], 0, -a[0]],
            [-a[1], a[0], 0]]


def rotationMatrix3D(axisVector, theta):
    U = axisVector.dot(axisVector.T)
    Uc = SrdMath.crossProductMatrix3D(axisVector)
    return np.cos(theta) * np.eye(3) + np.sin(theta) * Uc + (1 - np.cos(theta)) * U


def weightedPseudoinverse(A, B, alpha1, alpha2):
    '''
    Implements a weighted pseudoinverse based on Tikhonov
    regularization.
    The problem is stated as follows:
    Ax + By = z;
    find x and y, with different weights placed on them.

    it is solved as follows:
    M := [A B]; T = [I1*alpha1, 0; 0, I2*alpha2],
    where I1 and I2 are identity matrices with sizes dependant on x
    and y.

    [x; y] = (M'*M + T'*T) * M'z;
    This function returns P = pinv(M'*M + T'*T) * M' - a weighted pseudoinverse
    '''

    n1 = A.shape[1]
    n2 = B.shape[1]

    M = np.concatenate(A, B)

    T = block_diag(np.eye(n1) * alpha1, np.eye(n2) * alpha2)
    return np.linalg.pinv(M.T * M + T) * M.T


def findPlaceInArray(array, value, searchType="closest"):
    '''
    finds the index into an array where the closest value to the
    requested one lies
    '''
    A = array - value
    if searchType == "closest":
        A = np.abs(A)
        return np.amin(A)
    elif searchType == "closest smaller":
        A[A > 0] = -np.inf
        if np.max(A) == -np.inf:
            return -1
        else:
            return np.amax(A)
    elif searchType == "closest bigger":
        A[A < 0] = np.inf
        if np.min(A) == np.inf:
            return -1
        else:
            return np.amin(A)


def convexHull(points):
    return points[ConvexHull(points).vertices]


def clampTo2pi(theta):
    return np.clip(theta, 0, 2 * np.pi)


def callForAllCombinations(inputOne, inputTwo, functionReference):
    inputTuples = product(inputOne, inputTwo)
    if SrdMath.useParallel:
        with Pool(SrdMath.numberOfWorkers) as pool:
            outputs = pool.starmap(functionReference, iterable=inputTuples)
    else:
        outputs = []
        for inputTuple in inputTuples:
            outputs.append(functionReference(inputTuple[0], inputTuple[1]))

    xInputs, yInputs = zip(*product(inputOne, inputTwo))
    return list(xInputs), list(yInputs), list(outputs)


def parallelizedSimplification(X):

    rows, columns = len(X),len(X[0])

    if SrdMath.useParallel:
        with Pool(SrdMath.numberOfWorkers) as pool:
            simplifiedX = pool.starmap(simplify, iterable=chain(X))

        simplifiedX = [simplifiedX[columns*i: columns * (i + 1)] for i in range(rows)]
    else:
        simplifiedX = []
        for row in X:
            simplifiedRow = []
            for elem in row:
                simplifiedRow.append(simplify(elem))
            simplifiedX.append(simplifiedRow)

    return  simplifiedX


def derivative(x, q, v):
    return jacobian(x,q)*v


def matrixDerivative(X,q,v):
    rows, columns = len(X),len(X[0])

    dXtuples = chain.from_iterable([zip(x,q,v) for x in X])

    if SrdMath.useParallel:
        with Pool(SrdMath.numberOfWorkers) as pool:
            dX = pool.starmap(SrdMath.derivative, iterable=dXtuples)

        dX = [dX[columns*i: columns * (i + 1)] for i in range(rows)]
    else:
        dX = []
        for row in X:
            dXrow = []
            for x,_q,_v in zip(row,q,v):
                dXrow.append(jacobian(x,_q)*_v)
            dX.append(dXrow)

    return  dX

def rotation_transform(v1,v2):
  #https://math.stackexchange.com/a/3219491
  v = np.cross(v1, v2)
  u = v/np.linalg.norm(v)
  c = np.dot(v1, v2)
  h = (1 - c)/(1 - c**2)

  vx, vy, vz = v
  return [[c + h*vx**2, h*vx*vy - vz, h*vx*vz + vy,0],
        [h*vx*vy+vz, c+h*vy**2, h*vy*vz-vx,0],
        [h*vx*vz - vy, h*vy*vz + vx, c+h*vz**2,0],
        [0,0,0,1]]