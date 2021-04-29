import pybullet as p
import time
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
    

iiwaLinks = getLinkArrayFromURDF(os.path.abspath("./SrdPy/examples/iiwa/iiwa14.urdf"),True)
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

constraint = engine.links["iiwa_link_0"].absoluteFollower[0]

constraint = constraint #horizontal stack of row vectors

description_constraints = generateSecondDerivativeJacobians(engine,
                                                            task=constraint,
                                                            functionName_Task="g_Constraint",
                                                            functionName_TaskJacobian="g_Constraint_Jacobian",
                                                            functionName_TaskJacobianDerivative="g_Constraint_Jacobian_derivative",
                                                            casadi_cCodeFilename="g_Constraints",
                                                            path="./iiwa/Constraints")

handlerConstraints = ConstraintsModelHandler(description_constraints, engine.dof)

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
#plotIKTable(ikModelHandler, timeTable, IKTable)

ikSolutionHandler = IKSolutionHandler(ikModelHandler, handlerIK_taskSplines, timeTable, IKTable, "linear")

timeStep = 1./500
p.connect(p.GUI)
p.setGravity(0,0,-9.8)
p.setTimeStep(timeStep)
p.getCameraImage(480,320)
p.setRealTimeSimulation(0)

urdfFlags = p.URDF_USE_SELF_COLLISION
iiwa = p.loadURDF(os.path.abspath("./SrdPy/examples/iiwa/iiwa14.urdf"),[0,0,0.48],[0,0,0,1], flags = urdfFlags,useFixedBase=True)

jointIds=[]
jointNames = []

for j in range (p.getNumJoints(iiwa)):
    p.changeDynamics(iiwa,j,linearDamping=0, angularDamping=0)
    info = p.getJointInfo(iiwa,j)
    jointName = info[1]
    jointType = info[2]
    if jointType==p.JOINT_REVOLUTE:
        jointIds.append(j)
        jointNames.append(jointName)

jointNames = [link.joint.name for link in iiwaChain.linkArray[1:]]
jointIds = [id for name,id in zip(jointNames,jointIds) if name in jointNames]


stateHandler = BulletStateHandler(iiwa, jointIds)
gcModelEvaluator = GCModelEvaluatorHandler(handlerGeneralizedCoordinatesModel, stateHandler)

tf = ikSolutionHandler.timeExpiration

n = handlerGeneralizedCoordinatesModel.dofConfigurationSpaceRobot

dt = 0.001
tf = ikSolutionHandler.timeExpiration

simulationHandler = SimulationHandler(np.arange(0, tf, dt),True,timeStep)

desiredStateHandler = DesiredStateHandler(ikSolutionHandler, simulationHandler)

stateSpaceHandler = StateConverterGenCoord2StateSpaceHandler(stateHandler)

desiredStateSpaceHandler = StateConverterGenCoord2StateSpaceHandler(desiredStateHandler)

inverseDynamicsHandler = IDVanillaDesiredTrajectoryHandler(desiredStateHandler, gcModelEvaluator,simulationHandler)
linearModelEvaluator = LinearModelEvaluatorHandler(handlerGeneralizedCoordinatesModel, handlerLinearizedModel,
                                                    stateHandler, inverseDynamicsHandler, False)
computedTorqueController = LQRControllerHandler(stateSpaceHandler, desiredStateSpaceHandler, linearModelEvaluator,
                                         simulationHandler,
                                         inverseDynamicsHandler, 10 * np.eye(linearModelEvaluator.dofRobotStateSpace),
                                         np.eye(linearModelEvaluator.dofControl))


preprocessingHandlers = [desiredStateHandler, stateSpaceHandler, desiredStateSpaceHandler, gcModelEvaluator]
controllerHandlers = [inverseDynamicsHandler,linearModelEvaluator,computedTorqueController]

simulationHandler.preprocessingHandlersArray = preprocessingHandlers
simulationHandler.controllerArray = controllerHandlers


while(1):
    simulationHandler.step()
    u = np.array(computedTorqueController.u).T.tolist()[0]
    for j in range (len(u)):
        p.setJointMotorControl2(iiwa, jointIds[j], p.TORQUE_CONTROL, 0, force=u[j])
        
    p.stepSimulation()

    time.sleep(timeStep)


