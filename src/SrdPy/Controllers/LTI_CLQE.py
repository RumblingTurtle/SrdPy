from casadi.casadi import horzcat, vertcat
import numpy as np
from SrdPy.Math import svd_suit
from control.matlab import lqr
#Required input fields
#
#system.tol - tolerance
#
# dx/xdt = A*x + B*u + g + F*l
# y = C x
# G dx/xdt = 0
#

class LTI_System:
    def __init__(self,A,B,C,G,g,controllerCost,observerCost,x_desired,x_initial,tol):
        self.A = A
        self.B = B
        self.C = C
        self.G = G
        self.g = g

        self.controllerCost = controllerCost
        self.observerCost   = observerCost

        self.x_desired = x_desired
        self.x_initial = x_initial

        self.tol = tol


def LTI_CLQE(system):

    A = system.A
    B = system.B
    C = system.C
    G = system.G
    g = system.g

    controllerCost = system.controllerCost
    observerCost   = system.observerCost

    x_desired = system.x_desired
    x_initial = system.x_initial

    tol = system.tol

    size_x = A.shape[1]
    size_u = B.shape[1]
    size_y = C.shape[0]
    size_l = G.shape[0]

    G = svd_suit(G, tol)
    N = G.null 
    R = G.row_space #E = [N, R]

    ##################################################################
    # Derivative-State Constraints and Effective States 
    #D1*dx/dt + D2*x = 0
    dof = int(size_x/2)
    D1 = np.hstack([np.eye(dof), np.zeros((dof,dof))])
    D2 = np.hstack([np.zeros((dof,dof)), np.eye(dof)])

    GG = vertcat(horzcat(G.M, np.zeros((size_l, size_x))),horzcat(D1, D2))
    GG = svd_suit(GG, tol)

    V = GG.null[size_x:, :]

    temp = svd_suit(np.linalg.pinv(N.T@A)@(N.T@A)@V@np.linalg.pinv(V) @ R, tol)
    R_used = temp.orth

    E = np.hstack([N, R_used])

    size_z    = N.shape[1]
    size_zeta = R_used.shape[1]
    size_xi   = size_z+size_zeta

    ##################################################################

    Kz  = lqr(N.T@A@N, N.T@B, controllerCost['Q'], controllerCost['R'])
    Kzeta = np.linalg.pinv(N.T@B) @ N.T@A@R_used
    K = np.hstack((Kz[0], Kzeta))

    N1 = np.hstack([N, np.zeros((size_x, size_zeta))])
    A1 = N1.T@A@E
    B1 = N1.T@B

    L = lqr(N1.T@A@E, E.T@C.T, observerCost['Q'], observerCost['R'])
    L = L[0].T

    ##################################################################
    ### IC
    class IC:
        def __init__(self):
            self.x    = None
            self.z    = None
            self.zeta = None

            self.xi_estimate_random = None

    initialConditions = IC()
    initialConditions.x    = x_initial
    initialConditions.z    = N.T     @x_initial
    initialConditions.zeta = R_used.T@x_initial

    initialConditions.xi_estimate_random = 0.01*np.random.randn(size_xi, 1) 

    zeta = initialConditions.zeta

    ##################################################################
    ### desired


    class Desired:
        class Derivation:
            def __init__(self):
                self.M = None
                self.N = None
                self.zu_particular = None

                self.z = None
                self.u = None
                self.z_particular = None
                self.projector = None

        def __init__(self):
            self.x      = None
            self.z      = None
            self.zeta   = None

            self.derivation = Desired.Derivation()

            self.z_corrected = None

            self.zeta_corrected = None
            self.x_corrected    = None
            self.u_corrected    = None

    desired = Desired()
    desired.x      = x_desired
    desired.z      = N.T@x_desired
    desired.zeta   = R_used.T@x_desired 

    desired.derivation.M = svd_suit(np.hstack(N.T@A@N, N.T@B), tol)
    desired.derivation.N = desired.derivation.M.null
    desired.derivation.zu_particular = -desired.derivation.M.pinv @ N.T@g

    desired.derivation.z = np.hstack([np.eye(size_z),           np.zeros(size_z, size_u)])
    desired.derivation.u =  np.hstack([np.zeros(size_u, size_z), np.eye(size_u)])
    desired.derivation.z_particular = -desired.derivation.z * desired.derivation.M.pinv * N.T@g
    desired.derivation.Projector = (desired.derivation.z@desired.derivation.N) @ np.linalg.pinv(desired.derivation.z @ desired.derivation.N)

    desired.z_corrected = desired.derivation.z_particular + desired.derivation.projector @ (desired.z - desired.derivation.z_particular)

    z_des = desired.z_corrected
    x_des = N@z_des + R_used@zeta

    if np.linalg.norm( (np.eye(size_z) - (N.T@B)*np.linalg.pinv(N.T@B)) *(N.T@A@N@z_des + N.T@g) ) < tol:
        u_des = -np.linalg.pinv(N.T@B) *(N.T@A@N@z_des + N.T@g)
    else:
        raise Warning('cannot create a node')
    

    desired.zeta_corrected = zeta
    desired.x_corrected    = x_des
    desired.u_corrected    = u_des

    ##################################################################
    ### Projected LTI in z-xi coordinates

    class ClosedLoop:
        class Coords:
            def __init__(self):
                self.Matrix = None
                self.Vector = None
                self.ode_func = None
                self.Y0 = None
                self.M = None
        
        
        def __init__(self):
            self.z_xi = ClosedLoop.Coords()
            self.x_xi = ClosedLoop.Coords()
    
    closed_loop = ClosedLoop()
    closed_loop.z_xi.Matrix = np.vstack(np.hstack([N.T@A@N,   -N.T@B@K]),
                                np.hstack([L@C@N,     (N1.T@A@E - N1.T@B@K - L@C@E)]))

    closed_loop.z_xi.Vector = np.vstack(np.hstack(([N.T@A@R_used@zeta + N.T @B@Kz@z_des + N.T @B@u_des + N.T @g]),
                                        np.hstack([L@C@R_used@zeta  + N1.T@B@Kz@z_des + N1.T@B@u_des + N1.T@g])))


    closed_loop.z_xi.ode_fnc = lambda y:  closed_loop.z_xi.Matrix @ y + closed_loop.z_xi.Vector

    closed_loop.z_xi.Y0 = np.vstack([initialConditions.z,initialConditions.xi_estimate_random])

    ##################################################################
    ### LTI in x-xi coordinates

    closed_loop.x_xi.M = np.vstack([np.hstack([np.eye(size_x), np.zeros(size_x, size_xi),   -R]),
                                    np.hstack([np.zeros(size_xi, size_x), np.eye(size_xi,  size_xi),    np.zeros(size_xi, size_l)]) ,
                                    np.hstack([G.self,                 np.zeros(size_l, size_xi),    np.zeros(size_l, size_l)])])

    iM = np.linalg.pinv(closed_loop.x_xi.M)
    iM11 = iM[:(size_x+size_xi), :(size_x+size_xi)]

    closed_loop.x_xi.Matrix = iM11@np.vstack(np.hstack(A,-B@K),np.hstack(L@C,    (N1.T@A@E - N1.T@B@K - L@C@E)))


    closed_loop.x_xi.Vector = iM11@np.vstack([B@Kz@z_des + B@u_des + g, N1.T@B@Kz@z_des + N1.T@B@u_des + N1.T@g])


    closed_loop.x_xi.ode_fnc = lambda y: closed_loop.x_xi.Matrix @ y + closed_loop.x_xi.Vector         
    closed_loop.x_xi.Y0 = np.vstack(initialConditions.x,initialConditions.xi_estimate_random)

    ##################################################################

    class Output:
        def __init__(self):
            self.sizes = None 
            self.initialConditions = None
            self.desired = None
            self.closed_loop = None
            self.matrices = None

    output = Output()

    output.sizes = {'size_x': size_x, 'size_u': size_u, 'size_y': size_y, 'size_l': size_l,
        'size_z': size_z, 'size_zeta': size_zeta, 'size_xi': size_xi}

    output.initialConditions = initialConditions
    output.desired = desired
    output.closed_loop = closed_loop

    output.matrices = {'G': G, 'N': N, 'R': R, 'R_used': R_used, 'E': E, 'Kz': Kz, 'Kzeta': Kzeta, 'K': K, 'L': L, 'N1': N1}

    return output

