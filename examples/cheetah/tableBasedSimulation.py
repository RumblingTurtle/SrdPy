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

cheetahLinks = getLinkArrayFromURDF(os.path.abspath(os.path.dirname(sys.argv[0])+"/cheetah/urdf/cheetah.urdf"),True)
cheetahChain = Chain(cheetahLinks)
remap = [
'trunk',
'FL_hip',
'FL_thigh',
'FL_calf',
'FL_foot',
'FR_hip',
'FR_thigh',
'FR_calf',
'FR_foot',
'RL_hip',
'RL_thigh',
'RL_calf',
'RL_foot',
'RR_hip',
'RR_thigh',
'RR_calf',
'RR_foot'
]

cheetahChain.remapGenCoords(remap)
print(cheetahChain)
initialPosition = np.zeros(18)

engine = SymbolicEngine(cheetahChain.linkArray)

deriveJacobiansForlinkArray(engine)
H = deriveJSIM(engine)
np.set_printoptions(linewidth=1000)

iN, dH = deriveGeneralizedInertialForces_dH(engine, H)
g = deriveGeneralizedGravitationalForces(engine)
d = deriveGeneralizedDissipativeForcesUniform(engine, 1)
T = deriveControlMapFloating(engine)


description_gen_coord_model = generateDynamicsGeneralizedCoordinatesModel(engine,
                                                                            H=H,
                                                                            c=(iN + g + d),
                                                                            T=T,
                                                                            functionName_H="g_dynamics_H",
                                                                            functionName_c="g_dynamics_c",
                                                                            functionName_T="g_dynamics_T",
                                                                            casadi_cCodeFilename="g_dynamics_generalized_coordinates",
                                                                            path="./cheetah/Dynamics")

handlerGeneralizedCoordinatesModel = GeneralizedCoordinatesModelHandler(description_gen_coord_model)


description_linearization = generateDynamicsLinearization(engine,
                                                            H=H,
                                                            c=(iN + g + d),
                                                            T=T,
                                                            functionName_A="g_linearization_A",
                                                            functionName_B="g_linearization_B",
                                                            functionName_c="g_linearization_c",
                                                            casadi_cCodeFilename="g_dynamics_linearization",
                                                            path="./cheetah/Linearization")
                                                            
handlerLinearizedModel = LinearizedModelHandler(description_linearization)


constraint1 = engine.links["RL_calf"].absoluteFollower[0]
constraint2 = engine.links["RR_calf"].absoluteFollower[0]
constraint3 = engine.links["FL_calf"].absoluteFollower[0]
constraint4 = engine.links["FR_calf"].absoluteFollower[0]
constraint5 = engine.links["FR_thigh"].absoluteFollower[0]

constraint = np.hstack([[constraint1[0],constraint1[2]], constraint2,constraint3])
print("constraint size is: ", constraint.size)

description_constraints,F,dF = generateSecondDerivativeJacobians(engine,
                                                            task=constraint,
                                                            functionName_Task="g_Constraint",
                                                            functionName_TaskJacobian="g_Constraint_Jacobian",
                                                            functionName_TaskJacobianDerivative="g_Constraint_Jacobian_derivative",
                                                            casadi_cCodeFilename="g_Constraints",
                                                            path="./cheetah/Constraints")

handlerConstraints = ConstraintsModelHandler(description_constraints, engine.dof)

CoM = cheetahChain.getCoM()

task = np.hstack([constraint5,constraint4,constraint])
print("task size is: ", task.size)


description_IK,F,dF = generateSecondDerivativeJacobians(engine,
                                                task=task,
                                                functionName_Task="g_InverseKinematics_Task",
                                                functionName_TaskJacobian="g_InverseKinematics_TaskJacobian",
                                                functionName_TaskJacobianDerivative="g_InverseKinematics_TaskJacobian_derivative",
                                                casadi_cCodeFilename="g_InverseKinematics",
                                                path="./cheetah/InverseKinematics")

ikModelHandler = IKModelHandler(description_IK, engine.dof, task.shape[0])


IC_task = ikModelHandler.getTask(initialPosition)

ikOffset = np.zeros(IC_task.shape[0])
ikOffset[0] = 0.2
ikOffset[1] = 0
ikOffset[2] = 0.2

ikOffset[3] = 0.2
ikOffset[4] = 0
ikOffset[5] = 0.1

zeroOrderDerivativeNodes = np.hstack((IC_task,IC_task+ikOffset))

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


n = handlerGeneralizedCoordinatesModel.dofConfigurationSpaceRobot

ikSolutionHandler = IKSolutionHandler(ikModelHandler, handlerIK_taskSplines, timeTable, IKTable, "linear")


tf = ikSolutionHandler.timeExpiration

n = handlerGeneralizedCoordinatesModel.dofConfigurationSpaceRobot

A_table, B_table, c_table, x_table, u_table, dx_table = generateLinearModelTable(handlerGeneralizedCoordinatesModel,handlerLinearizedModel,ikSolutionHandler,timeTable)


N_table, G_table, F_table = generateConstraiedModelTable(handlerConstraints,handlerGeneralizedCoordinatesModel,x_table)

Q = 100*np.eye(2 * n)
R = 0.01*np.eye(handlerGeneralizedCoordinatesModel.dofControl)
count = A_table.shape[0]
K_table = generateCLQRTable(A_table, B_table, np.tile(Q, [count,1, 1]), np.tile(R, [ count, 1, 1]),N_table)


AA_table, cc_table = generateCloseLoopTable(A_table, B_table, c_table, K_table, x_table, u_table)

ode_fnc_handle = ClosedLoopConstrainedLinearSystemOdeFunctionHandler(AA_table, cc_table,G_table, F_table, timeTable)

x0 = np.hstack((initialPosition, np.zeros(initialPosition.shape[0])))

sol = solve_ivp(ode_fnc_handle, [0, tf], x0, t_eval=timeTable,method="LSODA")

time_table_0 = sol.t
solution_tape = sol.y.T

ax = plotGeneric(time_table_0,solution_tape,figureTitle="",ylabel="ODE", plot=True)
ax = plotGeneric(timeTable,x_table,ylabel="linearmodel", plot=True)

ax = plotGeneric(timeTable,solution_tape[:,:n],figureTitle="position",ylabel="q", plot=True)
ax = plotGeneric(timeTable,solution_tape[:,n:2*n],figureTitle="velocity",ylabel="v", plot=True)

with open('anim_array.npy', 'wb') as f:
    np.save(f, solution_tape[:,:n])

initialPosition = np.zeros(18)
blank_chain = deepcopy(cheetahChain)
blank_chain.update(initialPosition)

with open('anim_array.npy', 'rb') as f:
    q = np.load(f)

vis = Visualizer()
vis.animate(blank_chain,q,framerate=0.1,showMeshes=True)

input() 