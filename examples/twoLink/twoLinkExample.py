from os import stat
from SrdPy import save
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

from SrdPy import Chain
import numpy as np
import matplotlib.pyplot as plt

ground =  GroundLink()

link1 = Link(name='Link_1', order=1,
                inertia=np.diag([1/12*10*0.5, 1/12*10*0.5, 0.1*1/12*10*0.5]), mass=10,
                relativeBase=[0, 0, 0], relativeFollower=[[0,0,0.5]], relativeCoM=[0, 0, 0.25])


joint1 = JointPivotX(name="Joint_1", childLink=link1, parentLink=ground, parentFollowerNumber=0,
                        usedGeneralizedCoordinates=np.array([0]), usedControlInputs=[0],
                        defaultJointOrientation=np.eye(3))

link2 = Link(name='Link_2', order=2,
                inertia=np.diag([1/12*10*0.5, 1/12*10*0.5, 0.1*1/12*10*0.5]), mass=10,
                relativeBase=[0, 0, 0], relativeFollower=[[0,0.5,0]], relativeCoM=[0, 0.25, 0])


joint2 = JointPivotX(name="Joint_2", childLink=link2, parentLink=link1, parentFollowerNumber=0,
                        usedGeneralizedCoordinates=np.array([1]), usedControlInputs=[1],
                        defaultJointOrientation=np.eye(3))




linkArray = [ground,link1, link2]

initialPosition = np.array([0,0])
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

d = deriveGeneralizedDissipativeForcesUniform(engine,1) 
T = deriveControlMap(engine)

c = iN+g+d

description_gen_coord_model = generateDynamicsGeneralizedCoordinatesModel(engine,
                                                                        H=H,
                                                                        c=c,
                                                                        T=T,
                                                                        functionName_H="g_dynamics_H",
                                                                        functionName_c="g_dynamics_c",
                                                                        functionName_T="g_dynamics_T",
                                                                        casadi_cCodeFilename="g_dynamics_generalized_coordinates",
                                                                        path="./TwoLink/Dynamics")

gcModel = GeneralizedCoordinatesModelHandler(description_gen_coord_model)

constraint = engine.linkArray[2].absoluteFollower[0][2]

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
                                                        path="./TwoLink/Linearization")
linearizedModel = LinearizedModelHandler(description_linearization)
task = vertcat(engine.q[0],constraint)

desc,_,_ = generateSecondDerivativeJacobians(engine,
                                                task=task,
                                                functionName_Task="g_InverseKinematics_task",
                                                functionName_TaskJacobian="g_InverseKinematics_taskJacobian",
                                                functionName_TaskJacobianDerivative="g_InverseKinematics_taskJacobian_derivative",
                                                casadi_cCodeFilename="g_InverseKinematics",
                                                path="./TwoLink/InverseKinematics",
                                                recalculate=True,
                                                useJIT=False)

ikModelHandler = IKModelHandler(desc, engine.dof, 2)

IC_task = ikModelHandler.getTask(initialPosition)

zeroOrderDerivativeNodes = [[IC_task[0], IC_task[0] - 0.1],
                            [IC_task[1], IC_task[1]]
                            ]

firstOrderDerivativeNodes = [[0, 0],
                            [0, 0]]

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

qva = ikSolutionHandler.getPositionVelocityAcceleration(0)

stateHandler = StateHandler(qva[0],qva[1])

C = [[1,0,0,0],
     [0,1,0,0]]

gcModelEvaluator = GCModelEvaluatorHandler(gcModel,stateHandler,True)

linearModelEvaluator = LinearModelEvaluatorHandlerConstrained(gcModel,linearizedModel,constraintsHandler, stateHandler,[])

##########################################################################
dt = 0.001
tf = ikSolutionHandler.timeExpiration
timeHandler = TimeHandler(np.arange(0, tf, dt))

desiredStateHandler = DesiredStateHandler(ikSolutionHandler, timeHandler)

stateSpaceHandler = StateConverterGenCoord2StateSpaceHandler(stateHandler)

desiredStateSpaceHandler = StateConverterGenCoord2StateSpaceHandler(desiredStateHandler)

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
    timeHandler 
    ])


for i in range(len(timeHandler.timeLog)-1):
    updater.update()

##############################################################
##############################################################
fig, (ax1, ax2) = plt.subplots(1, 2)
ax1.plot(timeHandler.timeLog[:-1], stateLoggerPlant.q, '--', linewidth=3)

ax1.plot(timeHandler.timeLog[:-1], stateLoggerObserver.q, linewidth=1)

ax2.plot(timeHandler.timeLog[:-1], stateLoggerPlant.v, '--',linewidth=3)

ax2.plot(timeHandler.timeLog[:-1], stateLoggerObserver.v,linewidth=1)
plt.show()