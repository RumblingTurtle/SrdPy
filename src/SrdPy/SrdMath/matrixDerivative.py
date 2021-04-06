from itertools import chain
from multiprocessing import Pool

import SrdPy.SrdMath


def matrixDerivative(X,q,v):
    rows, columns = len(X),len(X[0])

    dXtuples = chain.from_iterable([zip(x,q,v) for x in X])

    if SrdMath.useParallel:
        with Pool(SrdMath.numberOfWorkers) as pool:
            dX = pool.starmap(SrdPy.SrdMath.derivative.derivative, iterable=dXtuples)

        dX = [dX[columns*i: columns * (i + 1)] for i in range(rows)]
    else:
        dX = []
        for row in X:
            dXrow = []
            for x,_q,_v in zip(row,q,v):
                dXrow.append(jacobian(x,_q)*_v)
            dX.append(dXrow)

    return  dX