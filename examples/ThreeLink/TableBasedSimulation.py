from SrdPy.TableGenerators.generateConstraiedLinearModelTable import generateConstraiedLinearModelTable
from SrdPy.TableGenerators.generateLinearModelTable import generateLinearModelTable
from SrdPy.Handlers.getGCModelEvaluatorHandler import GCModelEvaluatorHandler
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

def tableBasedSimulation():
    groundLink = GroundLink()

    link1 = Link(name="Link1", order=1,
                    inertia=np.diag([1 / 12 * 10 * 0.5, 1 / 12 * 10 * 0.5, 0.1 * 1 / 12 * 10 * 0.5]), mass=10,
                    relativeBase=[0, 0, 0], relativeFollower=[[0, 0, 0.5]], relativeCoM=[0, 0, 0.25])

    link2 = Link(name="Link2", order=1,
                    inertia=np.diag([1 / 12 * 10 * 0.5, 1 / 12 * 10 * 0.5, 0.1 * 1 / 12 * 10 * 0.5]), mass=10,
                    relativeBase=[0, 0, 0], relativeFollower=[[0, 0, 0.5]], relativeCoM=[0, 0, 0.25])

    link3 = Link(name="Link3", order=1,
                    inertia=np.diag([1 / 12 * 10 * 0.5, 1 / 12 * 10 * 0.5, 0.1 * 1 / 12 * 10 * 0.5]), mass=10,
                    relativeBase=[0, 0, 0], relativeFollower=[[0, 0, 0.5]], relativeCoM=[0, 0, 0.25])

    genCoordIndex = 0

    newCoordIndices = [0]
    jointGto1 = JointPivotX(name="GroundToFirst", childLink=link1, parentLink=groundLink, parentFollowerNumber=0,
                               usedGeneralizedCoordinates=newCoordIndices, usedControlInputs=newCoordIndices,
                               defaultJointOrientation=np.eye(3))

    newCoordIndices = [1]
    joint1to2 = JointPivotX(name="1To2", childLink=link2, parentLink=link1, parentFollowerNumber=0,
                               usedGeneralizedCoordinates=newCoordIndices, usedControlInputs=newCoordIndices,
                               defaultJointOrientation=np.eye(3))

    newCoordIndices = [2]
    joint2to3 = JointPivotX(name="2To3", childLink=link3, parentLink=link2, parentFollowerNumber=0,
                               usedGeneralizedCoordinates=newCoordIndices, usedControlInputs=newCoordIndices,
                               defaultJointOrientation=np.eye(3))

    initialPosition = np.array([np.pi / 4, -2 * np.pi / 3, 1 * np.pi / 5])
    linkArray = [groundLink, link1, link2, link3]

    chain =Chain(linkArray)
    chain.name = "Three link chain"
    
    blank_chain = deepcopy(chain)
    blank_chain.update(initialPosition)

    print(chain)

    engine = SymbolicEngine(chain.linkArray)

    deriveJacobiansForlinkArray(engine)
    H = deriveJSIM(engine)

    iN, dH = deriveGeneralizedInertialForces_dH(engine, H)
    g = deriveGeneralizedGravitationalForces(engine)
    d = deriveGeneralizedDissipativeForcesUniform(engine, 1)
    T = deriveControlMap(engine)
    # NaiveControlMap


    description_gen_coord_model = generateDynamicsGeneralizedCoordinatesModel(engine,
                                                                              H=H,
                                                                              c=(iN + g + d),
                                                                              T=T,
                                                                              functionName_H="g_dynamics_H",
                                                                              functionName_c="g_dynamics_c",
                                                                              functionName_T="g_dynamics_T",
                                                                              casadi_cCodeFilename="g_dynamics_generalized_coordinates",
                                                                              path="./Dynamics")

    handlerGeneralizedCoordinatesModel = getGeneralizedCoordinatesModelHandlers(description_gen_coord_model)

    description_linearization = generateDynamicsLinearization(engine,
                                                              H=H,
                                                              c=(iN + g + d),
                                                              T=T,
                                                              functionName_A="g_linearization_A",
                                                              functionName_B="g_linearization_B",
                                                              functionName_c="g_linearization_c",
                                                              casadi_cCodeFilename="g_dynamics_linearization",
                                                              path="./Linearization")
                                                              
    handlerLinearizedModel = getLinearizedModelHandlers(description_linearization)

    constraint = engine.linkArray[3].absoluteFollower[0][2]

    description_constraints = generateSecondDerivativeJacobians(engine,
                                                                task=constraint,
                                                                functionName_Task="g_Constraint",
                                                                functionName_TaskJacobian="g_Constraint_Jacobian",
                                                                functionName_TaskJacobianDerivative="g_Constraint_Jacobian_derivative",
                                                                casadi_cCodeFilename="g_Constraints",
                                                                path="./Constraints")
                                                                
    handlerConstraints = getConstraintsModelHandlers(description_constraints, engine.dof)

    task = vertcat(vertcat(engine.q[0], engine.q[1]), constraint)

    description_IK = generateSecondDerivativeJacobians(engine,
                                                       task=task,
                                                       functionName_Task="g_InverseKinematics_Task",
                                                       functionName_TaskJacobian="g_InverseKinematics_TaskJacobian",
                                                       functionName_TaskJacobianDerivative="g_InverseKinematics_TaskJacobian_derivative",
                                                       casadi_cCodeFilename="g_InverseKinematics",
                                                       path="./InverseKinematics")

    IKModelHandler = getIKModelHandler(description_IK, engine.dof, task.shape[0])
    IC_task = IKModelHandler.getTask(initialPosition)

    zeroOrderDerivativeNodes = [[IC_task[0], IC_task[0] - 0.15],
                                [IC_task[1], IC_task[1] + 0.15 ],
                                [IC_task[2], IC_task[2]]]

    firstOrderDerivativeNodes = [[0, 0],
                                 [0, 0],
                                 [0, 0]]

    secondOrderDerivativeNodes = [[0, 0],
                                  [0, 0],
                                  [0, 0]]

    timeOfOneStage = 2
    timeEnd = (len(zeroOrderDerivativeNodes[1]) - 1) * timeOfOneStage + 1
    nodeTimes = np.arange(start=0, stop=timeEnd, step=timeOfOneStage)

    handlerIK_taskSplines = getIKtaskSplinesHandler(nodeTimes,
                                                    zeroOrderDerivativeNodes, firstOrderDerivativeNodes,
                                                    secondOrderDerivativeNodes)

    timeTable = np.arange(handlerIK_taskSplines.timeStart, handlerIK_taskSplines.timeExpiration + 0.01, 0.01)
    IKTable = inverseKinematicsGenerateTable(IKModelHandler, handlerIK_taskSplines, initialPosition, timeTable)

    IKSolutionHandler = getIKSolutionHandler(IKModelHandler, handlerIK_taskSplines, timeTable, IKTable, "linear")

    stateHandler = getStateHandler(initialPosition,np.zeros(initialPosition.shape))

    timeHandler = TimeHandler()

    desiredStateHandler = getDesiredStateHandler(IKSolutionHandler,timeHandler)


    tf = IKSolutionHandler.timeExpiration

    n = handlerGeneralizedCoordinatesModel.dofConfigurationSpaceRobot

    A_table, B_table, c_table, x_table, u_table, dx_table = generateLinearModelTable(handlerGeneralizedCoordinatesModel,handlerLinearizedModel,IKSolutionHandler,timeTable)

    N_table, G_table= generateConstraiedModelTable(handlerConstraints,x_table,[])

    Q = 100*np.eye(2 * n)
    R = 0.01*np.eye(handlerGeneralizedCoordinatesModel.dofControl)
    count = A_table.shape[2]
    K_table = generateCLQRTable(A_table, B_table, np.matlib.repmat(Q, [1, 1, count]), np.matlib.repmat(R, [1, 1, count]), N_table)


    AA_table, cc_table = generateCloseLoopTable(A_table, B_table, c_table, K_table, x_table, u_table)

    ode_fnc_handle = getClosedLoopLinearSystemOdeFunctionHandler(AA_table, cc_table, timeTable)


    x0 = np.hstack(initialPosition, np.zeros(initialPosition.shape[0]))

    #time_table_0, solution_tape = ode45(ode_fnc_handle, [0, tf], x0)

