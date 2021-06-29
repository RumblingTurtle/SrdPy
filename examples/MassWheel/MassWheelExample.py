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

from SrdPy import Chain
import numpy as np

groundLink = GroundLink()

link1 = Link(name="Link1", order=1,
                inertia=np.diag([0, 0.003792, 0]), mass=0.67,
                relativeBase=[0, 0, 0], relativeFollower=[[0, 0, -0.38]], relativeCoM=[0, 0, -0.3134])

link2 = Link(name="Link2", order=2,
                inertia=np.diag([0, 0.0017, 0]), mass=0,
                relativeBase=[0, 0, 0], relativeFollower=[[0, 0, 0.5]], relativeCoM=[0, 0, 0])

joint1to2 = JointFloatingBase_XZ_plane(name="GroundTo1", childLink=link1, parentLink=groundLink, parentFollowerNumber=0,
                        usedGeneralizedCoordinates=np.array([0,1,-2]), usedControlInputs=[],
                        defaultJointOrientation=np.eye(3))

joint1to2 = JointPivotY(name="1To2", childLink=link2, parentLink=link1, parentFollowerNumber=0,
                        usedGeneralizedCoordinates=np.array([3]), usedControlInputs=[0],
                        defaultJointOrientation=np.eye(3))


initialPosition = np.array([0, 0, pi, -np.pi/2])
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


d = np.array([0,0,0,0.00015])
T = np.array([0,0,0,0.061])


description_gen_coord_model = generateDynamicsGeneralizedCoordinatesModel(engine,
                                                                        H=H,
                                                                        c=(iN + g + d),
                                                                        T=T,
                                                                        functionName_H="g_dynamics_H",
                                                                        functionName_c="g_dynamics_c",
                                                                        functionName_T="g_dynamics_T",
                                                                        casadi_cCodeFilename="g_dynamics_generalized_coordinates",
                                                                        path="./MassWheel/Dynamics")

handlerGeneralizedCoordinatesModel = GeneralizedCoordinatesModelHandler(description_gen_coord_model)

constraint = np.array([engine.q[0],engine.q[1]])


description_constraints,F,dF = generateSecondDerivativeJacobians(engine,
                                                            task=constraint,
                                                            functionName_Task="g_Constraint",
                                                            functionName_TaskJacobian="g_Constraint_Jacobian",
                                                            functionName_TaskJacobianDerivative="g_Constraint_Jacobian_derivative",
                                                            casadi_cCodeFilename="g_Constraints",
                                                            path="./MassWheel/Constraints",
                                                            recalculate=False,
                                                            useJIT=False)
handlerConstraints = ConstraintsModelHandler(description_constraints, engine.dof)


description_linearization = generateDynamicsLinearizationConstrained(engine,
                                                        H=H,
                                                        c=(iN + g + d),
                                                        T=T,
                                                        F=F,
                                                        dF=dF,
                                                        functionName_A="g_linearization_A",
                                                        functionName_B="g_linearization_B",
                                                        functionName_c="g_linearization_c",
                                                        casadi_cCodeFilename="g_dynamics_linearization",
                                                        path="./MassWheel/Linearization")
                                                        
handlerLinearizedModel = LinearizedModelHandler(description_linearization)

save(handlerGeneralizedCoordinatesModel,"handlerGeneralizedCoordinatesModel")
save(handlerLinearizedModel,"handlerLinearizedModel")
save(handlerConstraints,"handlerConstraints")