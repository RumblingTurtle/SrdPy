def crossProductMatrix3D(a):
    return [[0, -a[2], a[1]],
            [a[2], 0, -a[0]],
            [-a[1], a[0], 0]]