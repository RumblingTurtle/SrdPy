from SrdPy import save
from SrdPy.LinksAndJoints import *
from SrdPy.Handlers import *
from SrdPy.InverseKinematics import *
from SrdPy.SymbolicUtils import *
from SrdPy.Loggers import *
from SrdPy.DynamicSolvers import *
from SrdPy.Controllers import *
from SrdPy.TableGenerators import generateLinearModelTable
from SrdPy.Visuals import Visualizer
from SrdPy import SymbolicEngine
from SrdPy import plotGeneric
from copy import deepcopy
from casadi import *
import matplotlib.pyplot as plt
from SrdPy import Chain
import numpy as np

groundLink = GroundLink()

link1 = Link(name="Link1", order=1,
                inertia=np.diag([0, 0.003792, 0]), mass=0.67,
                relativeBase=[0, 0, 0], relativeFollower=[[0, 0, -0.38]], relativeCoM=[0, 0, -0.3134])

link2 = Link(name="Link2", order=2,
                inertia=np.diag([0, 0.0017, 0]), mass=0,
                relativeBase=[0, 0, 0], relativeFollower=[[0, 0, 0.5]], relativeCoM=[0, 0, 0])

joint1to2 = JointPivotY(name="GroundTo1", childLink=link1, parentLink=groundLink, parentFollowerNumber=0,
                        usedGeneralizedCoordinates=np.array([0]), usedControlInputs=[],
                        defaultJointOrientation=np.eye(3))

joint1to2 = JointPivotY(name="1To2", childLink=link2, parentLink=link1, parentFollowerNumber=0,
                        usedGeneralizedCoordinates=np.array([1]), usedControlInputs=[0],
                        defaultJointOrientation=np.eye(3))


initialPosition = np.array([0, -np.pi/2])
linkArray = [groundLink, link1, link2]

chain = Chain(linkArray)

chain.name = "Three link chain"

blank_chain = deepcopy(chain)
blank_chain.update(initialPosition)

print(chain)



engine = SymbolicEngine(chain.linkArray)

deriveJacobiansForlinkArray(engine)
H = deriveJSIM(engine)

iN, dH = deriveGeneralizedInertialForces_dH(engine, H)
g = deriveGeneralizedGravitationalForces(engine)


d = np.array([0,0.00015])
T = np.array([0,0.061])


description_gen_coord_model = generateDynamicsGeneralizedCoordinatesModel(engine,
                                                                        H=H,
                                                                        c=(iN + g + d),
                                                                        T=T,
                                                                        functionName_H="g_dynamics_H",
                                                                        functionName_c="g_dynamics_c",
                                                                        functionName_T="g_dynamics_T",
                                                                        casadi_cCodeFilename="g_dynamics_generalized_coordinates",
                                                                        path="./MassWheel/Dynamics")

gcModelHandler = GeneralizedCoordinatesModelHandler(description_gen_coord_model)


description_linearization = generateDynamicsLinearization(engine,
                                                        H=H,
                                                        c=(iN + g + d),
                                                        T=T,
                                                        functionName_A="g_linearization_A",
                                                        functionName_B="g_linearization_B",
                                                        functionName_c="g_linearization_c",
                                                        casadi_cCodeFilename="g_dynamics_linearization",
                                                        path="./Zonotope/Linearization")
                                                        
handlerLinearizedModel = LinearizedModelHandler(description_linearization)

task = np.array([engine.q[0],engine.q[1]])

description_task,F,dF = generateSecondDerivativeJacobians(engine,
                                                            task=task,
                                                            functionName_Task="g_Task",
                                                            functionName_TaskJacobian="g_Task_Jacobian",
                                                            functionName_TaskJacobianDerivative="g_Task_Jacobian_derivative",
                                                            casadi_cCodeFilename="g_Task",
                                                            path="./Zonotope/Task",
                                                            recalculate=True,
                                                            useJIT=False)

ikModelHandler = IKModelHandler(description_task, engine.dof,task.shape[0])


IC_task = ikModelHandler.getTask(initialPosition)

zeroOrderDerivativeNodes = [[IC_task[0], IC_task[0] - 0.15],
                            [IC_task[1], IC_task[1] + 0.15 ]
                            ]

firstOrderDerivativeNodes = [[0, 0],
                            [0, 0]
                            ]

secondOrderDerivativeNodes = [[0, 0],
                            [0, 0]]

timeOfOneStage = 2
timeEnd = (len(zeroOrderDerivativeNodes[1]) - 1) * timeOfOneStage + 1
nodeTimes = np.arange(start=0, stop=timeEnd, step=timeOfOneStage)

handlerIK_taskSplines = IKtaskSplinesHandler(nodeTimes,
                                                zeroOrderDerivativeNodes, firstOrderDerivativeNodes,
                                                secondOrderDerivativeNodes)

timeTable = np.arange(handlerIK_taskSplines.timeStart, handlerIK_taskSplines.timeExpiration + 0.01, 0.01)
IKTable = generateIKTable(ikModelHandler, handlerIK_taskSplines, initialPosition, timeTable)

#plotIKTable(ikModelHandler, timeTable, IKTable)

ikSolutionHandler = IKSolutionHandler(ikModelHandler, handlerIK_taskSplines, timeTable, IKTable, "linear")


timeHandler = TimeHandler()

desiredStateHandler = DesiredStateHandler(ikSolutionHandler,timeHandler)

tf = ikSolutionHandler.timeExpiration
tf = 0.1
timeTable = np.arange(0, tf, 0.01)
timeHandler = TimeHandler(timeTable)

desiredStateHandler = DesiredStateHandler(ikSolutionHandler, timeHandler)


n = gcModelHandler.dofConfigurationSpaceRobot


A_table, B_table, c_table, x_table, u_table, dx_table = generateLinearModelTable(gcModelHandler,handlerLinearizedModel,ikSolutionHandler,timeTable)


A_table_p = A_table + np.random.randn(*A_table.shape)*10**(-5)
B_table_p = B_table + np.random.randn(*B_table.shape)*10**(-5)

AA=np.stack((A_table,A_table_p), axis=3)
BB=np.stack((B_table,B_table_p), axis=3)


K_table, G_table, T_table = explicitZonotopeTable(AA,BB,zonotope_order=10,cost_weights_G=1,cost_weights_T=1,cost_weights_b=1)

dof = initialPosition.shape[0]
Count = G_table.shape[0]

for j in range(dof):
    for i in range(Count):
        drawZonotope(np.vstack([G_table[i, j, :],G_table[i,j+dof, :]]), np.zeros((2, 1)))
plt.show()
