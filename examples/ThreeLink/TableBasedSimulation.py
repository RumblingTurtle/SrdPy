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

constraint = engine.linkArray[3].absoluteFollower[0][2]

description_constraints = generateSecondDerivativeJacobians(engine,
                                                            task=constraint,
                                                            functionName_Task="g_Constraint",
                                                            functionName_TaskJacobian="g_Constraint_Jacobian",
                                                            functionName_TaskJacobianDerivative="g_Constraint_Jacobian_derivative",
                                                            casadi_cCodeFilename="g_Constraints",
                                                            path="./Constraints")
                                                            
handlerConstraints = ConstraintsModelHandler(description_constraints, engine.dof)

task = vertcat(vertcat(engine.q[0], engine.q[1]), constraint)

description_IK = generateSecondDerivativeJacobians(engine,
                                                    task=task,
                                                    functionName_Task="g_InverseKinematics_Task",
                                                    functionName_TaskJacobian="g_InverseKinematics_TaskJacobian",
                                                    functionName_TaskJacobianDerivative="g_InverseKinematics_TaskJacobian_derivative",
                                                    casadi_cCodeFilename="g_InverseKinematics",
                                                    path="./InverseKinematics")

ikModelHandler = IKModelHandler(description_IK, engine.dof, task.shape[0])
IC_task = ikModelHandler.getTask(initialPosition)

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

handlerIK_taskSplines = IKtaskSplinesHandler(nodeTimes,
                                                zeroOrderDerivativeNodes, firstOrderDerivativeNodes,
                                                secondOrderDerivativeNodes)

timeTable = np.arange(handlerIK_taskSplines.timeStart, handlerIK_taskSplines.timeExpiration + 0.01, 0.01)
IKTable = generateIKTable(ikModelHandler, handlerIK_taskSplines, initialPosition, timeTable)

ikSolutionHandler = IKSolutionHandler(ikModelHandler, handlerIK_taskSplines, timeTable, IKTable, "linear")

stateHandler = StateHandler(initialPosition,np.zeros(initialPosition.shape))

timeHandler = TimeHandler()

desiredStateHandler = DesiredStateHandler(ikSolutionHandler,timeHandler)


tf = ikSolutionHandler.timeExpiration

n = handlerGeneralizedCoordinatesModel.dofConfigurationSpaceRobot

A_table, B_table, c_table, x_table, u_table, dx_table = generateLinearModelTable(handlerGeneralizedCoordinatesModel,handlerLinearizedModel,ikSolutionHandler,timeTable)

N_table, G_table= generateConstraiedModelTable(handlerConstraints,x_table,[])

Q = 100*np.eye(2 * n)
R = 0.01*np.eye(handlerGeneralizedCoordinatesModel.dofControl)
count = A_table.shape[0]
K_table = generateLQRTable(A_table, B_table, np.tile(Q, [count,1, 1]), np.tile(R, [ count, 1, 1]))


AA_table, cc_table = generateCloseLoopTable(A_table, B_table, c_table, K_table, x_table, u_table)

ode_fnc_handle = ClosedLoopLinearSystemOdeFunctionHandler(AA_table, cc_table, timeTable)

x0 = np.hstack((initialPosition, np.zeros(initialPosition.shape[0])))

sol = solve_ivp(ode_fnc_handle, [0, tf], x0)
time_table_0 = sol.t
solution_tape = sol.y.T

ax = plotGeneric(time_table_0,solution_tape,figureTitle="",ylabel="ODE")
ax = plotGeneric(timeTable,x_table,ylabel="linearmodel",old_ax = ax, plot=True)