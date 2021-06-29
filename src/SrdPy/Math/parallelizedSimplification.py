from itertools import chain
from multiprocessing import Pool


def parallelizedSimplification(X):

    rows, columns = len(X),len(X[0])

    if Math.useParallel:
        with Pool(Math.numberOfWorkers) as pool:
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