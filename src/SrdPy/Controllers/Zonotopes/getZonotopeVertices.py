import numpy as np

def getZonotopeVertices(G):
    n, k = G.shape

    #https://stackoverflow.com/questions/28111051/create-a-matrix-of-binary-representation-of-numbers-in-python
    a = np.arange(k, dtype=int)[np.newaxis,:]
    l = int(np.log2(k))
    b = np.arange(l, dtype=int)[::-1,np.newaxis]
    binaryCubeVertices = np.array(a & 2**b > 0, dtype=int)

    binaryCubeVertices = 2*binaryCubeVertices - np.ones(binaryCubeVertices.shape)

    L = binaryCubeVertices.shape[1]
    points = np.zeros(n, L)

    for i in range(L):
        points[:, i] = G*binaryCubeVertices[:, i]

    return points