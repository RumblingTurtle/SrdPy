from casadi import *

def deriveCmatrixViaChristoffel(symbolicEngine,H):
    print('* Derivation of Generalized Inertial Forces started')

    # H*ddq + c + g = T*u        ddq = dv/dt v = dq/dt
    #
    # c = 0.5 * dH/dt * v

    q = symbolicEngine.q
    v = symbolicEngine.v
    n = H.size()[0]
    C = SX.zeros(n,n)
    
    for i in range(n):
        for j in range(n):
            for k in range(n):
                G = 0.5 * jacobian( H[i, j], q[k] ) + \
                    0.5 * jacobian( H[i, k], q[j] ) - \
                    0.5 * jacobian( H[k, j], q[i] )
                
                G = simplify(G)
                C[i, j] = C[i, j] + G * v[k]
            
            C[i, j] = simplify(C[i, j])
    return C