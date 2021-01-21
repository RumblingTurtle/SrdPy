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

from SrdPy import Chain
import numpy as np

def threeLinkExample():
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
    blank_chain = deepcopy(chain)
    blank_chain.update(initialPosition)


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

    inversekinematicsGenerateTableTester(IKModelHandler, timeTable, IKTable)

    IKSolutionHandler = getIKSolutionHandler(IKModelHandler, handlerIK_taskSplines, timeTable, IKTable, "linear")

    stateHandler = getStateHandler(initialPosition, np.zeros(len(initialPosition)))
    gcModelEvaluator = getGCModelEvaluatorHandler(handlerGeneralizedCoordinatesModel, stateHandler)
    linearModelEvaluator = getLinearModelEvaluatorHandler(handlerGeneralizedCoordinatesModel, handlerLinearizedModel,
                                                          stateHandler, [], False)

    dt = 0.001
    tf = IKSolutionHandler.timeExpiration

    simulationHandler = getSimulationHandler(np.arange(0, tf, dt))

    desiredStateHandler = getDesiredStateHandler(IKSolutionHandler, simulationHandler)

    stateSpaceHandler = getStateConverterGenCoord2StateSpaceHandler(stateHandler)

    desiredStateSpaceHandler = getStateConverterGenCoord2StateSpaceHandler(desiredStateHandler)

#    inverseDynamicsHandler = getIDVanillaDesiredTrajectoryHandler(desiredStateHandler, gcModelEvaluator,
#                                                                  simulationHandler)
    inverseDynamicsHandler = getInverseDynamicsConstrained_QR(
                            desiredStateHandler,
                            handlerConstraints,
                            gcModelEvaluator,
                            simulationHandler)

    computedTorqueController = getComputedTorqueController(stateHandler, desiredStateHandler,
                                                           gcModelEvaluator, simulationHandler, inverseDynamicsHandler,
                                                           500 * np.eye(desiredStateHandler.dofRobot),
                                                           100 * np.eye(desiredStateHandler.dofRobot))
    

                            
#    LQRHandler = getLQRControllerHandler(stateSpaceHandler, desiredStateSpaceHandler, linearModelEvaluator,
#                                         simulationHandler,
#                                         inverseDynamicsHandler, 10 * np.eye(linearModelEvaluator.dofRobotStateSpace),
#                                         np.eye(linearModelEvaluator.dofControl))
    LQRHandler = getConstrainedLQRHandler(stateHandler,stateSpaceHandler, desiredStateSpaceHandler, linearModelEvaluator,handlerConstraints,
                                         simulationHandler,
                                         inverseDynamicsHandler, 10 * np.eye(linearModelEvaluator.dofRobotStateSpace),
                                         np.eye(linearModelEvaluator.dofControl))

    mainController = LQRHandler

    linearModelEvaluator.controllerHandler = inverseDynamicsHandler

    taylorSolverHandler = getConstrainedTaylorSolverHandler(stateHandler, mainController, gcModelEvaluator, simulationHandler,handlerConstraints)

    stateHandlerLogger = getStateLoggerHandler(stateHandler, simulationHandler)

    tickLogger = getProgressDisplayHandler(simulationHandler)

    preprocessingHandlers = [desiredStateHandler, stateSpaceHandler, desiredStateSpaceHandler, gcModelEvaluator]
    controllerHandlers = [inverseDynamicsHandler, linearModelEvaluator, LQRHandler]
    solverHandlers = [taylorSolverHandler]
    loggerHandlers = [stateHandlerLogger, tickLogger]

    simulationHandler.preprocessingHandlersArray = preprocessingHandlers
    simulationHandler.controllerArray = controllerHandlers
    simulationHandler.solverArray = solverHandlers
    simulationHandler.loggerArray = loggerHandlers

    simulationHandler.simulate()

    plotGeneric(simulationHandler.timeLog[:-1], stateHandlerLogger.q, figureTitle="Q", ylabel="q")
    plotGeneric(simulationHandler.timeLog[:-1], stateHandlerLogger.v, figureTitle="V", ylabel="v")
    vis = Visualizer()
    vis.animate(blank_chain,stateHandlerLogger.q,framerate=0.1)