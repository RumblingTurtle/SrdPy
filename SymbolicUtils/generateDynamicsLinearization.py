from casadi import *
from SrdPy import SymbolicEngine


def generateDynamicsLinearization(symbolicEngine:SymbolicEngine, H, c, T):
    # H*ddq + c = T*u;        ddq = dv/dt; v = dq/dt;
    # x = [q; v]
    #
    #
    # f= ddq = inv(H) * (T*u - c)
    #
    # dx/dt = A*x+B*u+lc
    #
    # A = [0      I]
    #     [df/dq  df/dv  ]
    #
    # B = [0           ]
    #     [inv(H)*T    ]
    #
    # lc = [0                             ]
    #      [inv(H)*c - df/dq*q -  df/dv*v ]
    #
    # df / dq = d(inv(H))/dq * (T*u - c) + d(T*u - c)/dq
    # df / dq = inv(H) * dH/dq * inv(H) * (T*u - c) + d(T*u - c)/dq
    #
    # df / dv = inv(H)* d(T*u - c)/dv


    q = symbolicEngine.q
    v = symbolicEngine.v
    u = symbolicEngine.u

    n = symbolicEngine.dof
    m = len(symbolicEngine.u)

    iH = SX.sym('iH', [n, n])

    TCq = SX.jacobian(T*u+c, q)
    TCv = SX.jacobian(T*u+c, v)


    dfdq = -iH*SX.reshape(SX.jacobian(H, q)*(iH*(T*u+c)), n, n) + TCq

    dfdv = iH * TCv

    A = [[zeros(n, n), eye(n)], [dfdq, dfdv]]

    B = [[zeros(n, m)],[iH*T]]

    linear_c = [SX.zeros(n, 1),[iH*c - dfdq*q - dfdv*v]]

    return A,B,linear_c