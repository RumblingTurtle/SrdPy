import unittest
from SrdPy.Tests import *
from SrdPy.Handlers import *
from SrdPy import SymbolicEngine
import numpy as np
from casadi import *
from SrdPy.InverseKinematics import *
from SrdPy.SymbolicUtils import *
import scipy.io as sio
import os
from SrdPy.Tests.Cheetah import *
from copy import deepcopy

class CheetahTests(unittest.TestCase):
    def __init__(self):
        cheetahChain = getCheetahChain()
        initialPosition = np.zeros(18)
        blank_chain = deepcopy(cheetahChain)
        blank_chain.update(initialPosition)
        
        engine = SymbolicEngine(cheetahChain.linkArray)

        deriveJacobiansForlinkArray(engine)
        H = deriveJSIM(engine)

        iN, dH = deriveGeneralizedInertialForces_dH(engine, H)
        g = deriveGeneralizedGravitationalForces(engine)
        d = deriveGeneralizedDissipativeForcesUniform(engine, 1)
        T = deriveControlMapFloating(engine)

        
        constraint3 = np.squeeze(engine.links["RL_calf"].absoluteFollower)
        CoM = engine.getCoM()

        task = np.hstack([constraint3,CoM])

        description_IK,F,dF = generateSecondDerivativeJacobians(engine,
                                                        task=task,
                                                        functionName_Task="g_InverseKinematics_Task",
                                                        functionName_TaskJacobian="g_InverseKinematics_TaskJacobian",
                                                        functionName_TaskJacobianDerivative="g_InverseKinematics_TaskJacobian_derivative",
                                                        casadi_cCodeFilename="g_InverseKinematics",
                                                        path="./cheetah/InverseKinematics")

        self.ikModelHandler = IKModelHandler(description_IK, engine.dof, task.shape[0])

    def testIKModelHandler(self):
        
        onesMat = sio.loadmat(os.path.abspath("./SrdPy/Tests/Cheetah/ikTaskOnes.mat"))['res']
        onesTask = np.array(self.ikModelHandler.getTask(np.ones([18,1])))

        twosMat = sio.loadmat(os.path.abspath("./SrdPy/Tests/Cheetah/ikTaskTwos.mat"))['res']
        twosTask = np.array(self.ikModelHandler.getTask(np.ones([18,1])*2))

        np.testing.assert_allclose(onesTask,onesMat)
        np.testing.assert_allclose(twosTask,twosMat)
        

