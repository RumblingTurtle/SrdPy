from casadi.casadi import  horzcat, vertcat
import numpy as np
from SrdPy.Math import svd_suit
from control.matlab import lqr,place

#Required input fields
#
#system.tol - tolerance
#
# dx/xdt = A*x + B*u + g + F*l
# y = C x
# G dx/xdt = 0
#

class LTI_System:
    def __init__(self,A,B,C,G,g,controllerSettings,observerSettings,x_desired,dx_desired,tol,x_initial=None):
        self.A = A
        self.B = B
        self.C = C
        self.G = G
        self.g = g

        self.controllerSettings = controllerSettings
        self.observerSettings   = observerSettings

        self.x_desired = x_desired
        self.dx_desired = dx_desired
        self.x_initial = x_initial

        self.tol = tol

class Cost:
     def __init__(self,Q,R):
         self.Q = Q 
         self.R = R


def LTI_CLQE(system,saveComputation):

    A = system.A
    B = system.B
    C = system.C
    G = system.G
    g = system.g

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
  
    NA = svd_suit(N.T@A,tol)
    temp = svd_suit(np.hstack([NA.null,N,GG.row_space[size_x:,:]]),tol)

    R_used = temp.left_null

    size_z    = N.shape[1]
    size_zeta = R_used.shape[1]
    size_chi   = size_z+size_zeta

    E = np.hstack([N, R_used])
    
    N1 = np.hstack([N, np.zeros((size_x, size_zeta))])

    
    ##################################################################
    class OutStruct:
        def __init__(self):
            self.matrix_chi = None
            self.matrix_u= None
            self.matrix_y= None
            self.vector = None
            self.map= None

    
    class Output:
        def __init__(self):
            self.sizes = None 
            self.initialConditions = None
            self.desired = None
            self.closed_loop = None
            self.matrices = None
            self.observer = OutStruct()
            self.controller = OutStruct()

    output = Output()

    output.sizes = {'size_x': size_x, 'size_u': size_u, 'size_y': size_y, 'size_l': size_l,
        'size_z': size_z, 'size_zeta': size_zeta, 'size_xi': size_chi}

    ##################################################################
    if system.controllerSettings is not None:
        if system.controllerSettings.Q.shape[1] == 1:
            costQ = np.diag(system.controllerSettings.Q[:size_z,:].squeeze())
        else:
            costQ = np.diag(system.controllerSettings.Q)
        
        if system.controllerSettings.R.shape[1] == 1:
            costR = np.diag(system.controllerSettings.R[:size_z].squeeze())
        else:
            costR = np.diag(system.controllerSettings.R)
        
        Kz = lqr(N.T@A@N, N.T@B, costQ, costR)[0]
    else:
        p = system.controllerSettings.poles[:N.shape[1]]
        Kz = place(N.T@A@N, N.T@B, p)
    
    if system.observerSettings is not None:
        if system.observerSettings.Q.shape[1] == 1:
            costQ = np.diag(system.observerSettings.Q[:size_z].squeeze())
        else:
            costQ = np.diag(system.observerSettings.Q)
        
        if system.observerSettings.R.shape[1] == 1:
            costR = np.diag(system.observerSettings.R[:size_z].squeeze())
        else:
            costR = np.diag(system.observerSettings.R)
        
        L = lqr((N1.T@A@E).T, E.T@C.T, costQ, costR)[0]
    else:
        p = system.observerSettings.poles[:E.shape[1]]
        L = place((N1.T@A@E).T, E.T@C.T, p)

    L = L.T
    


    Kzeta = np.linalg.pinv(N.T@B) @ N.T@A@R_used
    K = np.hstack((Kz, Kzeta))
    ##################################################################
    #observer structure

    output.controller.matrix_chi = K
    output.observer.matrix_chi = N1.T@A@E - L@C@E
    output.observer.matrix_u = N1.T@B
    output.observer.matrix_y = L
    output.observer.vector = N1.T@g

    output.observer.map = E

    output.matrices = {'G': G, 'N': N, 'R': R, 'R_used': R_used, 'E': E, 
        'Kz': Kz, 'Kzeta': Kzeta, 'K': K, 'L': L, 
        'N1': N1}
        
    if saveComputation==2:
        return output

    ##################################################################
    ### desired

    class Desired:
        class Derivation:
            def __init__(self):
                self.NB = None
                self.NB_projector= None
                self.M = None
                self.zdz_corrected = None

        def __init__(self):
            self.x = None
            self.dx = None 
            self.z   = None
            self.dz = None
            self.z_corrected= None
            self.dz_corrected = None
            self.u_corrected = None
            self.derivation = Desired.Derivation()



    desired = Desired()
    desired.x  = system.x_desired
    desired.dx = system.dx_desired
    desired.z = N.T@desired.x
    desired.dz = N.T@desired.dx 

    desired.derivation.NB = svd_suit(N.T@B,tol)
    desired.derivation.NB_projector = desired.derivation.NB.left_null @ desired.derivation.NB.left_null.T

    desired.derivation.M = svd_suit(np.hstack([-desired.derivation.NB_projector@N.T@A@N, desired.derivation.NB_projector]), tol)
    desired.derivation.zdz_corrected = desired.derivation.M.pinv@desired.derivation.NB_projector@N.T@g + desired.derivation.M.null@desired.derivation.M.null.T @ np.vstack([desired.z, desired.dz])

    desired.z_corrected  = desired.derivation.zdz_corrected[:size_z]
    desired.dz_corrected = desired.derivation.zdz_corrected[size_z:]

    if np.linalg.norm( desired.derivation.NB.left_null@desired.derivation.NB.left_null.T@
            (desired.dz_corrected - N.T@A@N@desired.z_corrected - N.T@g) ) < tol:

        desired.u_corrected = desired.derivation.NB.pinv @ (desired.dz_corrected - N.T@A@N@desired.z_corrected - N.T@g)
        output.controller.vector = desired.u_corrected

    else:
        raise Warning('cannot create a node')
    
    if saveComputation == 1:
        output.desired = desired
        return output

    ##################################################################
    ### IC

    class InitialConditions:
        def __init__(self):
            self.x 
            self.z
            self.zeta 
            self.chiEstimateRandom

    x_initial = system.x_initial

    initialConditions = InitialConditions()

    initialConditions.x    = x_initial
    initialConditions.z    = N.T@x_initial
    initialConditions.zeta = R_used.T@x_initial

    InitialConditions.chi_estimate_random = 0.01*np.random.randn(size_chi, 1) 

    output.initialConditions = initialConditions

    zeta = initialConditions.zeta

    u_des = desired.u_corrected
    z_des = desired.z_corrected 
    x_des = N@z_des + R_used@zeta

    desired.zeta_corrected = zeta
    desired.x_corrected    = x_des

    output.desired = desired

    ##################################################################
    ### Projected LTI in z-chi coordinates
    class ClosedLoop:
        class Coords:
            def __init__(self):
                self.matrix = None
                self.vector = None
                self.ode_func = None
                self.Y0 = None
                self.M = None
        
        
        def __init__(self):
            self.z_chi = ClosedLoop.Coords()
            self.x_chi = ClosedLoop.Coords()

    closed_loop = ClosedLoop()

    closed_loop.z_chi.matrix = np.vstack(np.hstack([N.T@A@N,   -N.T@B@K]),
                                np.hstack([L@C@N,     (N1.T@A@E - N1.T@B@K - L@C@E)]))
        
    closed_loop.z_chi.vector = np.vstack([N.T@A@R_used@zeta + N.T @B@Kz@z_des + N.T @B@u_des + N.T @g,
                                L@C@R_used@zeta  + N1.T@B@Kz@z_des + N1.T@B@u_des + N1.T@g])
                    

    closed_loop.z_chi.ode_fnc = lambda y:  closed_loop.z_chi.matrix @ y + closed_loop.z_xi.vector                       
    closed_loop.z_chi.Y0 = np.hstack([initialConditions.z,initialConditions.chi_estimate_random])

    ##################################################################
    ### LTI in x-chi coordinates


    closed_loop.x_chi.M = np.vstack([
                        np.hstack([np.eye(size_x), np.zeros((size_x, size_chi)),   -R]),
                        np.hstack([np.zeros((size_chi, size_x)), np.eye((size_chi,size_chi)),np.zeros((size_chi, size_l))]),
                        np.hstack([G.M,np.zeros((size_l, size_chi)),np.zeros((size_l, size_l))])
                        ])

    iM = np.linalg.pinv(closed_loop.x_chi.M)
    iM11 = iM[:(size_x+size_chi), :(size_x+size_chi)]

    closed_loop.x_chi.matrix = iM11@np.vstack([np.hstack([A,     -B@K]),
                                    np.hstack([L@C,    (N1.T@A@E - N1.T@B@K - L@C@E)])])
        

    closed_loop.x_chi.vector = iM11@np.vstack([B@Kz@z_des + B@u_des+ g,N1.T@B@Kz@z_des + N1.T@B@u_des + N1.T@g])       


    closed_loop.x_chi.ode_fnc = lambda y:  closed_loop.x_chi.matrix @ y + closed_loop.x_xi.vector
    closed_loop.x_chi.Y0 = np.vstack([initialConditions.x, initialConditions.chi_estimate_random]) 

    output.closed_loop = closed_loop

    return output
