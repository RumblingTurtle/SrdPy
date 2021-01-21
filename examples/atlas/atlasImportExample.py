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
from SrdPy.URDFUtils import getLinkArrayFromURDF
from SrdPy import Chain
import numpy as np
from SrdPy.Visuals import Visualizer
import os
def atlasImportExample():
    dynamicsFolderName = "atlas"
    print(os.getcwd())
    atlasLinks = getLinkArrayFromURDF(os.path.abspath("./SrdPy/examples/atlas/atlas_v5.urdf"),True)
    atlasChain = Chain(atlasLinks)
    atlasChain.update(np.array([0]*36))
    
    engine = SymbolicEngine(atlasChain.linkArray)

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
                                                                              path="./"+dynamicsFolderName+"/Dynamics")

    description_linearization = generateDynamicsLinearization(engine,
                                                              H=H,
                                                              c=(iN + g + d),
                                                              T=T,
                                                              functionName_A="g_linearization_A",
                                                              functionName_B="g_linearization_B",
                                                              functionName_c="g_linearization_c",
                                                              casadi_cCodeFilename="g_dynamics_linearization",
                                                              path="./"+dynamicsFolderName+"/Linearization")

    handlerGeneralizedCoordinatesModel = getGeneralizedCoordinatesModelHandlers(description_gen_coord_model)

    handlerLinearizedModel = getLinearizedModelHandlers(description_linearization)

