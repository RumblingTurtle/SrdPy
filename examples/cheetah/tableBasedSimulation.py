from SrdPy.URDFUtils import getLinkArrayFromURDF

from SrdPy.TableGenerators import generateConstraiedLinearModelTable
from SrdPy.TableGenerators import generateLinearModelTable
from SrdPy.LinksAndJoints import *
from SrdPy.Handlers import *
from SrdPy.InverseKinematics import *
from SrdPy.SymbolicUtils import *
from SrdPy.Loggers import *
from SrdPy.DynamicSolvers import *
from SrdPy.Controllers import *

from SrdPy.Visuals import Visualizer
from SrdPy import SymbolicEngine
from SrdPy import plotGeneric
from copy import deepcopy
from casadi import *

from SrdPy.TableGenerators import *
from SrdPy import Chain
import numpy as np
from scipy.integrate import solve_ivp
import os

def tableBasedSimulation():
    cheetahLinks = getLinkArrayFromURDF(os.path.abspath("./SrdPy/examples/cheetah/cheetah/urdf/cheetah.urdf"),True)
    cheetahChain = Chain(cheetahLinks)
    
    initialPosition = np.zeros(12)
    cheetahChain.update(initialPosition)

    engine = SymbolicEngine(cheetahChain.linkArray)

    deriveJacobiansForlinkArray(engine)
    H = deriveJSIM(engine)

    iN, dH = deriveGeneralizedInertialForces_dH(engine, H)
    g = deriveGeneralizedGravitationalForces(engine)
    d = deriveGeneralizedDissipativeForcesUniform(engine, 1)
    T = deriveControlMap(engine)


    description_gen_coord_model = generateDynamicsGeneralizedCoordinatesModel(engine,
                                                                              H=H,
                                                                              c=(iN + g + d),
                                                                              T=T,
                                                                              functionName_H="g_dynamics_H",
                                                                              functionName_c="g_dynamics_c",
                                                                              functionName_T="g_dynamics_T",
                                                                              casadi_cCodeFilename="g_dynamics_generalized_coordinates",
                                                                              path="./Dynamics")

    handlerGeneralizedCoordinatesModel = GeneralizedCoordinatesModelHandler(description_gen_coord_model)

    description_linearization = generateDynamicsLinearization(engine,
                                                              H=H,
                                                              c=(iN + g + d),
                                                              T=T,
                                                              functionName_A="g_linearization_A",
                                                              functionName_B="g_linearization_B",
                                                              functionName_c="g_linearization_c",
                                                              casadi_cCodeFilename="g_dynamics_linearization",
                                                              path="./Linearization")
                                                              
    handlerLinearizedModel = LinearizedModelHandler(description_linearization)

    constraint1 = engine.linkArray[4].absoluteFollower
    constraint2 = engine.linkArray[8].absoluteFollower
    constraint3 = engine.linkArray[12].absoluteFollower
    
    constraint = np.vstack((constraint1,constraint2))

    description_constraints = generateSecondDerivativeJacobians(engine,
                                                                task=constraint,
                                                                functionName_Task="g_Constraint",
                                                                functionName_TaskJacobian="g_Constraint_Jacobian",
                                                                functionName_TaskJacobianDerivative="g_Constraint_Jacobian_derivative",
                                                                casadi_cCodeFilename="g_Constraints",
                                                                path="./Constraints")
                                                                
    handlerConstraints = ConstraintsModelHandler(description_constraints, engine.dof)
    
    CoM = cheetahChain.getCoM()

    task = np.vstack((constraint3, CoM))

    description_IK = generateSecondDerivativeJacobians(engine,
                                                    task=task,
                                                    functionName_Task="g_InverseKinematics_Task",
                                                    functionName_TaskJacobian="g_InverseKinematics_TaskJacobian",
                                                    functionName_TaskJacobianDerivative="g_InverseKinematics_TaskJacobian_derivative",
                                                    casadi_cCodeFilename="g_InverseKinematics",
                                                    path="./InverseKinematics")

    ikModelHandler = IKModelHandler(description_IK, engine.dof, task.shape[0])

    IC_task = ikModelHandler.getTask(initialPosition)


    zeroOrderDerivativeNodes =[ [IC_task[0], IC_task[0]+0.04],
                                [IC_task[1], IC_task[1]+0.04],
                                [IC_task[2], IC_task[2]+0.03],
                                [IC_task[3], IC_task[3]+0.04],
                                [IC_task[4], IC_task[4]+0.05],
                                [IC_task[5], IC_task[5]+0.00]]

    firstOrderDerivativeNodes = [[0, 0],
                                 [0, 0],
                                 [0, 0],
                                 [0, 0],
                                 [0, 0],
                                 [0, 0]]

    secondOrderDerivativeNodes = [[0, 0],
                                  [0, 0],
                                  [0, 0],
                                  [0, 0],
                                  [0, 0],
                                  [0, 0]]


    timeOfOneStage = 2
    timeEnd = (len(zeroOrderDerivativeNodes[1]) - 1) * timeOfOneStage + 1
    nodeTimes = np.arange(start=0, stop=timeEnd, step=timeOfOneStage)


    handlerIK_taskSplines = IKtaskSplinesHandler(nodeTimes,
                                                    zeroOrderDerivativeNodes, 
                                                    firstOrderDerivativeNodes,
                                                    secondOrderDerivativeNodes)

   
    timeTable = np.arange(handlerIK_taskSplines.timeStart, handlerIK_taskSplines.timeExpiration + 0.01, 0.01)

    IKTable = generateIKTable(ikModelHandler, handlerIK_taskSplines, initialPosition, timeTable)
    plotIKTable(ikModelHandler, timeTable, IKTable)

""" 
    SRD_InverseKinematics_GenerateTable_tester(...
        'Handler_IK_Model', Handler_IK_Model, ...
        'TimeTable', TimeTable, ...
        'IK_Table', IK_Table);

    Handler_IK_Solution = SRD_get_handler__IK_solution__interp1(...
        'Handler_IK_Model_name', 'Handler_IK_Model', ...
        'Handler_IK_task_name', 'Handler_IK_task', ...
        'IK_Table', IK_Table, ...
        'TimeTable', TimeTable, ...
        'method', 'linear');%linear', 'nearest', 'next', 'previous', 'pchip', 'cubic', 'v5cubic', 'makima', or 'spline'
 """