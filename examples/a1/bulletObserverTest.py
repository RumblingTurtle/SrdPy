from SrdPy.URDFUtils import getLinkArrayFromURDF
import pybullet as p
import time
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

from SrdPy.Controllers.Observers.LTI_CLQE import Cost
from SrdPy.TableGenerators import *
from SrdPy import Chain
from SrdPy import Profiler
import numpy as np
from scipy.integrate import solve_ivp
import os

a1Links = getLinkArrayFromURDF(os.path.abspath("./SrdPy/examples/a1/urdf/a1.urdf"),True)
a1Chain = Chain(a1Links)

print(a1Chain)
initialPosition = np.zeros(18)

engine = SymbolicEngine(a1Chain.linkArray)

deriveJacobiansForlinkArray(engine)
H = deriveJSIM(engine)

iN, dH = deriveGeneralizedInertialForces_dH(engine, H)
g = deriveGeneralizedGravitationalForces(engine)
T = deriveControlMapFloating(engine)
d = deriveGeneralizedDissipativeForcesUniform(engine,1) 

c = iN+g+d

description_gen_coord_model = generateDynamicsGeneralizedCoordinatesModel(engine,
                                                                            H=H,
                                                                            c=c,
                                                                            T=T,
                                                                            functionName_H="g_dynamics_H",
                                                                            functionName_c="g_dynamics_c",
                                                                            functionName_T="g_dynamics_T",
                                                                            casadi_cCodeFilename="g_dynamics_generalized_coordinates",
                                                                            path="./a1/Dynamics")

gcModel = GeneralizedCoordinatesModelHandler(description_gen_coord_model)

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
                                                            path="./TwoLink/Constraints")
constraintsHandler = ConstraintsModelHandler(description_constraints, engine.dof)

description_linearization = generateDynamicsLinearizationConstrained(engine,
                                                        H=H,
                                                        c=c,
                                                        T=T,
                                                        F=F,
                                                        dF=dF,
                                                        functionName_A="g_linearization_A",
                                                        functionName_B="g_linearization_B",
                                                        functionName_c="g_linearization_c",
                                                        casadi_cCodeFilename="g_dynamics_linearization",
                                                        path="./a1/Linearization")
                                                            
linearizedModel = LinearizedModelHandler(description_linearization)




description_constraints,F,dF = generateSecondDerivativeJacobians(engine,
                                                            task=constraint,
                                                            functionName_Task="g_Constraint",
                                                            functionName_TaskJacobian="g_Constraint_Jacobian",
                                                            functionName_TaskJacobianDerivative="g_Constraint_Jacobian_derivative",
                                                            casadi_cCodeFilename="g_Constraints",
                                                            path="./a1/Constraints")

constraintsHandler = ConstraintsModelHandler(description_constraints, engine.dof)

CoM = a1Chain.getCoM()

task = np.hstack([constraint5,constraint4,constraint])
print("task size is: ", task.size)


description_IK,F,dF = generateSecondDerivativeJacobians(engine,
                                                task=task,
                                                functionName_Task="g_InverseKinematics_Task",
                                                functionName_TaskJacobian="g_InverseKinematics_TaskJacobian",
                                                functionName_TaskJacobianDerivative="g_InverseKinematics_TaskJacobian_derivative",
                                                casadi_cCodeFilename="g_InverseKinematics",
                                                path="./a1/InverseKinematics")

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
#plotIKTable(ikModelHandler, timeTable, IKTable)


n = gcModel.dofConfigurationSpaceRobot

ikSolutionHandler = IKSolutionHandler(ikModelHandler, handlerIK_taskSplines, timeTable, IKTable, "linear")

timeStep = 1./1000
p.connect(p.GUI)
p.setGravity(0,0,-9.8)
p.setTimeStep(timeStep)
p.getCameraImage(480,320)
p.setRealTimeSimulation(0)
plane = p.loadURDF("./SrdPy/examples/a1/plane.urdf")
urdfFlags = p.URDF_USE_SELF_COLLISION
a1 = p.loadURDF(os.path.abspath("./SrdPy/examples/a1/urdf/a1.urdf"),[0,0,0.45],[0,0,0,1], flags = urdfFlags,useFixedBase=False)

jointIds=[]
jointNames = []

for j in range (p.getNumJoints(a1)):
    p.changeDynamics(a1,j,linearDamping=0, angularDamping=0)
    info = p.getJointInfo(a1,j)
    jointName = info[1]
    jointType = info[2]
    if jointType==p.JOINT_REVOLUTE:
        jointIds.append(j)
        jointNames.append(jointName)

jointNames = [link.joint.name for link in a1Chain.linkArray[1:]]
jointIds = [id for name,id in zip(jointNames,jointIds) if name in jointNames]


stateHandler = BulletStateHandler(a1, jointIds,True)
gcModelEvaluator = GCModelEvaluatorHandler(gcModel, stateHandler)
linearModelEvaluator = LinearModelEvaluatorHandlerConstrained(gcModel,linearizedModel,constraintsHandler, stateHandler,[])

C = np.hstack([np.eye(a1Chain.dof),np.zeros((a1Chain.dof,a1Chain.dof))])

tf = ikSolutionHandler.timeExpiration

n = gcModel.dofConfigurationSpaceRobot
dt = 0.001
tf = ikSolutionHandler.timeExpiration

timeHandler = TimeHandler(np.arange(0, tf, dt),True,timeStep)

desiredStateHandler = DesiredStateHandler(ikSolutionHandler, timeHandler)

stateSpaceHandler = StateConverterGenCoord2StateSpaceHandler(stateHandler)

desiredStateSpaceHandler = StateConverterGenCoord2StateSpaceHandler(desiredStateHandler)
qva = ikSolutionHandler.getPositionVelocityAcceleration(0)

stateHandler = StateHandler(qva[0],qva[1])
observerStateHandler = StateSpaceHandler(vertcat(qva[0],qva[1]))
observerStateGenCoordHandler = StateConverterStateSpace2GenCoordHandler(observerStateHandler)
###########################################################################
measuredOutput = MeasuredOutputHandler(stateSpaceHandler,np.array(C))

inverseDynamicsHandler = InverseDynamicsConstrained_QR(desiredStateHandler,constraintsHandler,gcModel,timeHandler)

computedTorqueController = ComputedTorqueController(stateHandler,desiredStateHandler,gcModelEvaluator,
timeHandler,inverseDynamicsHandler,500*np.eye(desiredStateHandler.dofRobot),100*np.eye(desiredStateHandler.dofRobot))

lqrHandler = ConstrainedLQRController(stateHandler=stateHandler,stateSpaceHandler=stateSpaceHandler,constraintsHandler=constraintsHandler,
controlInputStateSpaceHandler=desiredStateSpaceHandler,linearizedModelHandler=linearModelEvaluator,timeHandler=timeHandler,
inverseDynamicsHandler=inverseDynamicsHandler,Q=10*np.eye(linearModelEvaluator.dofRobotStateSpace),R=np.eye(linearModelEvaluator.dofControl))

n = linearModelEvaluator.dofRobotStateSpace
m = linearizedModel.dofControl

controller  = Cost(100*np.ones((n, 1)), np.ones((m, 1)))
observer    = Cost(10*np.ones((n, 1)), np.ones((m, 1)))

CLQE_Handler = CLQE(timeHandler, desiredStateSpaceHandler,inverseDynamicsHandler,gcModel,linearModelEvaluator,constraintsHandler,C,controller,observer,10**-6)
##############################
# #
mainController = lqrHandler
linearModelEvaluator.controllerHandler = inverseDynamicsHandler
##############################
# #


constrainedObserverHandler = ConstrainedObserver(CLQE_Handler,observerStateHandler,measuredOutput,mainController,
inverseDynamicsHandler,desiredStateSpaceHandler,timeHandler,10**-6)

##############################
# #

taylorConstrainedSolver = ConstrainedTaylorSolverHandler(stateHandler,mainController,gcModelEvaluator,timeHandler,constraintsHandler)

# #
##############################
stateLoggerPlant = StateLoggerHandler(stateHandler,timeHandler,False)
stateLoggerObserver = StateLoggerHandler(observerStateGenCoordHandler,timeHandler,False)

simulationTickDisplay = SimulationTickDisplay(timeHandler, displayFreq=100)
##############################
# #

updater = HandlerUpdater([
    desiredStateHandler, 
    stateSpaceHandler, 
    desiredStateSpaceHandler, 
    measuredOutput, 
    gcModelEvaluator,
    inverseDynamicsHandler, 
    linearModelEvaluator, 
    lqrHandler, 
    CLQE_Handler, 
    constrainedObserverHandler, 
    observerStateGenCoordHandler, 
    taylorConstrainedSolver, 
    stateLoggerPlant, 
    stateLoggerObserver, 
    simulationTickDisplay,  
    ])


while(1):
    timeHandler.update()
    updater.update()
    u = np.array(lqrHandler.u).T.tolist()[0]

    for j in range (len(u[6:])):
        p.setJointMotorControl2(a1, jointIds[j], p.TORQUE_CONTROL, 0, force=u[j])
        
    p.stepSimulation()

    time.sleep(timeStep)


