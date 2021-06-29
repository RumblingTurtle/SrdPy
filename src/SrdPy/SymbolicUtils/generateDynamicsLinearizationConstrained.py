from casadi import *
from SrdPy import SymbolicEngine
from SrdPy.Math import matrixJacobianTimesVector
import pickle
import os
import sys

def generateDynamicsLinearizationConstrained(symbolicEngine:SymbolicEngine,
                                    H, c, T,F,dF,
                                    functionName_A,
                                    functionName_B,
                                    functionName_c,
                                    casadi_cCodeFilename,
                                    path,recalculate=False,useJIT=False):

    pathFolder = os.path.basename(path)
    picklePath = os.path.join(path,pathFolder+".pkl")

    if os.path.exists(picklePath) and not recalculate:
        with open(picklePath, 'rb') as f:
            modelDict = pickle.load(f)
        print("Loaded existing .so at "+path)
        return modelDict

    # H*ddq + c = T*u        ddq = dv/dt v = dq/dt
    # x = [q v]
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
    m = symbolicEngine.u.shape[0]
    k = F.size()[0]

    map = np.hstack([np.eye(n), np.zeros((n, k))])
    
    iM = SX.sym('iM', n+k, n+k)
    
    M = vertcat(horzcat(H, -F.T),
        horzcat(F, SX.zeros(k,k)))

    RHS = vertcat(T@u-c, -dF@v)
    
    Jq = jacobian(RHS, q)
    print('Simplifying TCq')
    Jq = simplify(Jq)
        
    Jv = jacobian(RHS, v)
    print('Simplifying TCv')
    Jv = simplify(Jv)
    
    
    dfdq = -map@iM@matrixJacobianTimesVector(M, q, iM@RHS) + map@iM@Jq

    print('Simplifying dfdq')
    dfdq = simplify(dfdq)
    
    dfdv = map@iM @ Jv
    print('Simplifying dfdv')
    dfdv = simplify(dfdv)
    
    A = vertcat(horzcat(SX.zeros(n, n), SX.eye(n)),
        horzcat(dfdq,        dfdv))
     
    B = vertcat(SX.zeros(n, m),map@iM@vertcat(T,SX.zeros(k, m)))

    print('Starting writing function for the '+functionName_A)
    g_linearization_A = Function(functionName_A,
                                 [symbolicEngine.q,symbolicEngine.v,symbolicEngine.u,iM], [A],
                                 ['q', 'v', 'u', 'iH'], ['A'])

    print('Starting writing function for the ' + functionName_B)
    g_linearization_B = Function(functionName_B,
                                 [symbolicEngine.q,symbolicEngine.v,iM], [B],
                                 ['q', 'v', 'iH'], ['B'])

    c_function_name = casadi_cCodeFilename+'.c'
    so_function_name = casadi_cCodeFilename+'.so'

    current_cwd = os.getcwd()
    if os.path.isdir(path):
        os.chdir(path)
    else:
        os.makedirs(path)
        os.chdir(path)

    CG = CodeGenerator(c_function_name)
    CG.add(g_linearization_A)
    CG.add(g_linearization_B)
    #CG.add(g_linearization_c)
    CG.generate()
    
    if not useJIT:
        command = ["gcc","-fPIC","-shared",c_function_name, "-o",so_function_name]
        print("Running gcc")

        import subprocess
        exitcode = subprocess.Popen(command).wait()
        if exitcode!=0:
            print("GCC compilation error")
            return {}

    os.chdir(current_cwd)
    print("Generated C code!")

    resultDict = {"functionName_A": functionName_A,
            "functionName_B": functionName_B,
            "functionName_c": functionName_c,
            "casadi_cCodeFilename": casadi_cCodeFilename,
            "path": path,
            "dofConfigurationSpaceRobot": n,
            "dofStateSpaceRobot": 2 * n,
            "dofControl": m,
            "useJIT":useJIT}

    with open(picklePath, 'wb') as f:
        pickle.dump(resultDict, f)

    return resultDict