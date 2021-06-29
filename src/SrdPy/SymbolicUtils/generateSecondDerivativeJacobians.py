from casadi import *
from SrdPy import SymbolicEngine
import pickle
import os

def generateSecondDerivativeJacobians(symbolicEngine:SymbolicEngine,
                                    task,
                                    functionName_Task,
                                    functionName_TaskJacobian,
                                    functionName_TaskJacobianDerivative,
                                    casadi_cCodeFilename,
                                    path,recalculate=True,useJIT=True):

    
    pathFolder = os.path.basename(path)
    picklePath = os.path.join(path,pathFolder+".pkl")
    
    if os.path.exists(picklePath):
        if not recalculate:
            with open(picklePath, 'rb') as f:
                modelDict = pickle.load(f)
            print("Loaded existing .so at "+path)
            return modelDict
            
    dofTask = task.shape[0]
    taskJacobian = jacobian(task,symbolicEngine.q)
    taskJacobianDerivative = jacobian(taskJacobian,symbolicEngine.q)@symbolicEngine.v
    taskJacobianDerivative = reshape(taskJacobianDerivative, taskJacobian.shape)

    print('Starting writing function for the '+functionName_Task)
    g_InverseKinematics_Task = Function(functionName_Task,
                                 [symbolicEngine.q], [task],
                                 ['q'], ['task'])

    print('Starting writing function for the ' + functionName_Task+ " task jacobian")
    g_InverseKinematics_TaskJacobian = Function(functionName_TaskJacobian,
                                 [symbolicEngine.q], [taskJacobian],
                                 ['q'], ['taskJacobian'])

    print('Starting writing function for the derivative of ' + functionName_Task)
    g_InverseKinematics_TaskJacobianDerivative = Function(functionName_TaskJacobianDerivative,
                                     [symbolicEngine.q,symbolicEngine.v], [taskJacobianDerivative],
                                     ['q', 'v'], ['taskJacobianDerivative'])

    c_function_name = casadi_cCodeFilename+'.c'
    so_function_name = casadi_cCodeFilename+'.so'

    current_cwd = os.getcwd()
    if os.path.isdir(path):
        os.chdir(path)
    else:
        os.makedirs(path)
        os.chdir(path)

    CG = CodeGenerator(c_function_name)
    CG.add(g_InverseKinematics_Task)
    CG.add(g_InverseKinematics_TaskJacobian)
    CG.add(g_InverseKinematics_TaskJacobianDerivative)
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
    resultDict = {"functionName_Task": functionName_Task,
            "functionName_TaskJacobian": functionName_TaskJacobian,
            "functionName_TaskJacobianDerivative": functionName_TaskJacobianDerivative,
            "casadi_cCodeFilename": casadi_cCodeFilename,
            "dofRobot":symbolicEngine.dof,
            "dofTask":dofTask,
            "path": path,
            "useJIT":useJIT}

    with open(picklePath, 'wb') as f:
        pickle.dump(resultDict, f)

    return resultDict,taskJacobian,taskJacobianDerivative