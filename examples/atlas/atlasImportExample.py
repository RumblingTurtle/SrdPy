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
from cProfile import Profile

dynamicsFolderName = "atlas"
atlasLinks = getLinkArrayFromURDF(os.path.abspath("./SrdPy/examples/atlas/atlas_v5.urdf"),True)
atlasChain = Chain(atlasLinks)


engine = SymbolicEngine(atlasChain.linkArray)

deriveJacobiansForlinkArray(engine)
H = deriveJSIM(engine)

iN, dH = deriveGeneralizedInertialForces_dH(engine, H)
g = deriveGeneralizedGravitationalForces(engine)
d = deriveGeneralizedDissipativeForcesUniform(engine, 1)
T = deriveControlMap(engine)
# NaiveControlMap

sparsedH = engine.get_dH()

n = H.size()[0]
print(n)
siN = SX.zeros(n,1)
for i in range(n):
    for j in range(n):
        for k in range(n):
                indx1 = k+i*n+j*(n**2)
                indx2 = j+i*n+k*(n**2)
                indx3 = i+k*n+j*(n**2)
                G = simplify(0.5 * sparsedH[indx1] + 0.5 @ sparsedH[indx2] -0.5 @ sparsedH[indx3])
                siN[i] = siN[i] + G @ engine.v[j] @ engine.v[k]

print(iN-siN)
'''
description_gen_coord_model_sparse = generateDynamicsGeneralizedCoordinatesModel(engine,
                                                                            H=engine.getH(),
                                                                            c=(iN + g + d),
                                                                            T=T,
                                                                            functionName_H="g_dynamics_H",
                                                                            functionName_c="g_dynamics_c",
                                                                            functionName_T="g_dynamics_T",
                                                                            casadi_cCodeFilename="g_dynamics_generalized_coordinates",
                                                                            path="./"+"atlasSparse"+"/Dynamics")

description_gen_coord_model = generateDynamicsGeneralizedCoordinatesModel(engine,
                                                                            H=H,
                                                                            c=(iN + g + d),
                                                                            T=T,
                                                                            functionName_H="g_dynamics_H",
                                                                            functionName_c="g_dynamics_c",
                                                                            functionName_T="g_dynamics_T",
                                                                            casadi_cCodeFilename="g_dynamics_generalized_coordinates",
                                                                            path="./"+dynamicsFolderName+"/Dynamics")
    
description_gen_coord_model_sparse = generateDynamicsGeneralizedCoordinatesModel(None,
                                                                            H=None,
                                                                            c=None,
                                                                            T=None,
                                                                            functionName_H="g_dynamics_H",
                                                                            functionName_c="g_dynamics_c",
                                                                            functionName_T="g_dynamics_T",
                                                                            casadi_cCodeFilename="g_dynamics_generalized_coordinates",
                                                                            path="./"+"atlasSparse"+"/Dynamics")

description_gen_coord_model = generateDynamicsGeneralizedCoordinatesModel(None,
                                                                            H=None,
                                                                            c=None,
                                                                            T=None,
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
                                                            
handlerLinearizedModel = LinearizedModelHandler(description_linearization)


handlerGeneralizedCoordinatesModelSparse = GeneralizedCoordinatesModelHandler(description_gen_coord_model_sparse)
handlerGeneralizedCoordinatesModel = GeneralizedCoordinatesModelHandler(description_gen_coord_model)

sparseH = engine.getH()
sparseH = Function("sH", [engine.q], [sparseH], ['q'], ['sH'])

sparsedH = engine.get_dH()
sparsedH = Function("sdH", [engine.q], [sparsedH], ['q'], ['sdH'])


Hf = Function("H", [engine.q], [H], ['q'], ['dH'])

Hjac = jacobian(reshape(H,29*29,1),engine.q)
dH = Function("dH", [engine.q], [Hjac], ['q'], ['dH'])
cName = "sparseComp.c"

CG = CodeGenerator(cName)
CG.add(sparseH)
CG.add(sparsedH)
CG.generate()     

cName2 = "sComp.c"
CG2 = CodeGenerator(cName2)
CG2.add(Hf)
CG2.add(dH)
CG2.generate()     


imp = Importer(cName,"clang")
imp2 = Importer(cName2,"clang")
sparseH = external("sH", imp)
Hf = external("H", imp2)

checkNum = 100000
print("H comp")
pr = Profile()
pr.enable()

for i in range(checkNum):
    sparseH(np.random.rand(29))

pr.disable()
pr.print_stats()

pr = Profile()
pr.enable()
for i in range(checkNum):
    Hf(np.random.rand(29))

pr.disable()
pr.print_stats()

sparsedH = external("sdH", imp)
dH = external("dH", imp2)

print("dH comp")
pr = Profile()
pr.enable()

for i in range(checkNum):
    sparsedH(np.random.rand(29))

pr.disable()
pr.print_stats()

pr = Profile()
pr.enable()
for i in range(checkNum):
    dH(np.random.rand(29))

pr.disable()
pr.print_stats()

print("ddH comp")
pr = Profile()
pr.enable()

for i in range(checkNum):
    sparseddH(np.random.rand(29))

pr.disable()
pr.print_stats()

pr = Profile()
pr.enable()
for i in range(checkNum):
    ddH(np.random.rand(29))

pr.disable()
pr.print_stats()
'''
