import numpy as np


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