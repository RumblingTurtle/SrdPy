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
from SrdPy import Profiler
import numpy as np
from scipy.integrate import solve_ivp
import os
    
from control import lqr
def my_generateLQRTable(A_table, B_table, Q_table, R_table):
    count = A_table.shape[0]
    n = A_table.shape[2]
    m = B_table.shape[2]

    K_table = np.zeros((count,m,n))

    for i in range(count):
        K, S, CLP =  lqr(A_table[i], B_table[i], Q_table[i], R_table[i])
        K_table[i] = K
        
    return K_table

iiwaLinks = getLinkArrayFromURDF(os.path.abspath("./iiwa14.urdf"),True)
iiwaChain = Chain(iiwaLinks)


print(iiwaChain)
initialPosition = np.zeros(7)
blank_chain = deepcopy(iiwaChain)
blank_chain.update(initialPosition)

engine = SymbolicEngine(iiwaChain.linkArray)

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
                                                                            path="./iiwa/Dynamics")

handlerGeneralizedCoordinatesModel = GeneralizedCoordinatesModelHandler(description_gen_coord_model)

description_linearization = generateDynamicsLinearization(engine,
                                                            H=H,
                                                            c=(iN + g + d),
                                                            T=T,
                                                            functionName_A="g_linearization_A",
                                                            functionName_B="g_linearization_B",
                                                            functionName_c="g_linearization_c",
                                                            casadi_cCodeFilename="g_dynamics_linearization",
                                                            path="./iiwa/Linearization")
                                                            
handlerLinearizedModel = LinearizedModelHandler(description_linearization)

constraint6 = engine.links["iiwa_link_6"].absoluteFollower[0]

task = constraint6[:2]
print("task size is: ", task.size)


description_IK = generateSecondDerivativeJacobians(engine,
                                                task=task,
                                                functionName_Task="g_InverseKinematics_Task",
                                                functionName_TaskJacobian="g_InverseKinematics_TaskJacobian",
                                                functionName_TaskJacobianDerivative="g_InverseKinematics_TaskJacobian_derivative",
                                                casadi_cCodeFilename="g_InverseKinematics",
                                                path="./iiwa/InverseKinematics")

ikModelHandler = IKModelHandler(description_IK, engine.dof, task.shape[0])

IC_task = ikModelHandler.getTask(initialPosition)

IC_task = np.reshape(IC_task,[1,2])

task_1 = np.array([[0.1],
                [0.3]])

task_2 = np.array([[0.3],
                [0.3]])

task_3 = np.array([[0.3],
                [0.1]])

zeroOrderDerivativeNodes = np.hstack((IC_task.T, task_1, task_2, task_3))


firstOrderDerivativeNodes = np.zeros(zeroOrderDerivativeNodes.shape)

secondOrderDerivativeNodes = np.zeros(zeroOrderDerivativeNodes.shape)


timeOfOneStage = 2
timeEnd = (len(zeroOrderDerivativeNodes[1]) - 1) * timeOfOneStage + 1
nodeTimes = np.arange(start=0, stop=timeEnd, step=timeOfOneStage)


handlerIK_taskSplines = IKtaskSplinesHandler(nodeTimes,
                                                zeroOrderDerivativeNodes, 
                                                firstOrderDerivativeNodes,
                                                secondOrderDerivativeNodes)


timeTable = np.arange(handlerIK_taskSplines.timeStart, handlerIK_taskSplines.timeExpiration + 0.01, 0.01)

IKTable = generateIKTable(ikModelHandler, handlerIK_taskSplines, initialPosition, timeTable, method="lsqnonlin")
plotIKTable(ikModelHandler, timeTable, IKTable)

ikSolutionHandler = IKSolutionHandler(ikModelHandler, handlerIK_taskSplines, timeTable, IKTable, "linear")

tf = ikSolutionHandler.timeExpiration

n = handlerGeneralizedCoordinatesModel.dofConfigurationSpaceRobot

A_table, B_table, c_table, x_table, u_table, dx_table = generateLinearModelTable(handlerGeneralizedCoordinatesModel,handlerLinearizedModel,ikSolutionHandler,timeTable)

Q = 10*np.eye(2 * n)
R = 0.1*np.eye(handlerGeneralizedCoordinatesModel.dofControl)
count = A_table.shape[0]

С = np.concatenate((np.eye(n), np.zeros((n, n))), axis=1)
#y = C*x


K_table = my_generateLQRTable(A_table, B_table, np.tile(Q, [count,1, 1]), np.tile(R, [ count, 1, 1]))
AA_table, cc_table = generateCloseLoopTable(A_table, B_table, c_table, K_table, x_table, u_table)
ode_fnc_handle = ClosedLoopLinearSystemOdeFunctionHandler(AA_table, cc_table, timeTable)

x0 = np.hstack((initialPosition, np.zeros(initialPosition.shape[0])))

sol = solve_ivp(ode_fnc_handle, [0, tf], x0, t_eval=timeTable,method="RK45")

time_table_0 = sol.t
solution_tape = sol.y.T

ax = plotGeneric(time_table_0,solution_tape,figureTitle="",ylabel="ODE")
ax = plotGeneric(timeTable,x_table,ylabel="linearmodel",old_ax = ax, plot=True)

ax = plotGeneric(timeTable,solution_tape[:,:n],figureTitle="position",ylabel="q", plot=True)
ax = plotGeneric(timeTable,solution_tape[:,n:2*n],figureTitle="velocity",ylabel="v", plot=True)

with open('anim_array.npy', 'wb') as f:
    np.save(f, solution_tape[:,:n])
    
chainLinks = getLinkArrayFromURDF(os.path.abspath("./iiwa14.urdf"),True)
chain = Chain(chainLinks)

print(chain)
blank_chain = deepcopy(chain)
blank_chain.update(initialPosition)
with open('anim_array.npy', 'rb') as f:
    q = np.load(f)

blank_chain.update(q[0])
plotGeneric(np.arange(q.shape[0]),q,plot=True)
vis = Visualizer()
vis.animate(blank_chain,q,framerate=0.1,showMeshes=True)