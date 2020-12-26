import unittest
from SrdPy.Tests import *
from SrdPy.SymbolicUtils import *
from SrdPy.Handlers import *
from SrdPy import SymbolicEngine
import numpy as np
from casadi import *
from SrdPy.SymbolicUtils import *

class ThreeLinkTests(unittest.TestCase):
    def __init__(self):
        self.initialPositions = [[1,1,1],[np.pi,0,np.pi],[np.pi,np.pi,np.pi]]
        self.chain = getThreeLinkChain()
        self.engine = SymbolicEngine(self.chain.linkArray)
        self.generateJSIM()
        self.deriveLinearizationVariables()

    def generateJSIM(self):
        deriveJacobiansForlinkArray(self.engine)
        self.H = deriveJSIM(self.engine)


    def deriveLinearizationVariables(self):
        self.iN, self.dH = deriveGeneralizedInertialForces_dH(self.engine, self.H)
        self.g = deriveGeneralizedGravitationalForces(self.engine)
        self.d = deriveGeneralizedDissipativeForcesUniform(self.engine, 1)
        self.T = deriveControlMap(self.engine)
        self.c =(self.iN + self.g + self.d)

    def testJSIM(self):
        hfunc = Function("hfunc",[self.engine.q], [self.H],
                                            ['q'], ['h'])
        correctHMatrices = [[[14.9877, 7.44004, 1.19686],
                            [7.44004, 5.93409, 1.71704],
                            [1.19686, 1.71704, 1.04167]],

                           [[13.125, 4.58333, -1.45833],
                            [4.58333, 2.08333, -0.208333],
                            [-1.45833, -0.208333, 1.04167]],

                           [[3.125, -0.416667, 1.04167],
                            [-0.416667, 2.08333, -0.208333],
                            [1.04167, -0.208333, 1.04167]]
                           ]
        print("Testing JSIM")

        for testIdx,(initialPosition,correctH) in enumerate(zip(self.initialPositions,correctHMatrices)):

            npVal = np.array(DM(hfunc(initialPosition)))
            correctNpVal = np.array(correctH)
            np.testing.assert_allclose(npVal,correctNpVal,rtol=9)
            print("JSIM test {}/{} complete".format(testIdx+1,len(self.initialPositions)))
        print()

    def testModelVectorC(self):
        cfunc = Function("cfunc",[self.engine.q,self.engine.v], [self.c],
                                            ['q','v'], ['c'])

        vVec = [3,2,2]
        correctCVectors = [[49.1737, 81.6759, 37.983],[-3, -2, -2],[-3, -2, -2]]
        print("Testing c vector")
        for testIdx, (initialPosition, correctH) in enumerate(zip(self.initialPositions, correctCVectors)):
            npVal = np.array(DM(cfunc(initialPosition,vVec)))
            correctNpVal = np.array(correctH)
            np.testing.assert_allclose(np.squeeze(npVal.T),correctNpVal,rtol=9)
            print("C vector test {}/{} complete".format(testIdx + 1, len(self.initialPositions)))
        print()

    def testLinearizationMatrixA(self):

        description_linearization = generateDynamicsLinearization(self.engine,
                                                              H=self.H,
                                                              c=self.c,
                                                              T=self.T,
                                                              functionName_A="g_linearization_A",
                                                              functionName_B="g_linearization_B",
                                                              functionName_c="g_linearization_c",
                                                              casadi_cCodeFilename="g_dynamics_linearization",
                                                              path="./Linearization")

        handlerLinearizedModel = getLinearizedModelHandlers(description_linearization)
        correctMatricesA = [ [[     0,     0,     0,  1.0000,     0,     0],
                              [     0,     0,     0,     0,  1.0000,     0],
                              [     0,     0,     0,     0,     0,  1.0000],
                              [  37.2676, -43.9649, -13.8975,  -8.6517,  -6.0387,  -1.6207],
                              [  -93.8968, -77.7804, -46.4216,  12.9734,  3.3731,  -1.3585],
                              [  78.9000,  31.8026, -16.5657,  5.2005,  11.4760,  3.1415]],
                            [[0, 0, 0, 1.0000, 0, 0],
                             [0, 0, 0, 0, 1.0000, 0],
                             [0, 0, 0, 0, 0, 1.0000],
                             [-171.5000, -59.0000, 104.5000, -0.4849, 1.0194, -0.4751],
                             [-49.0000, -26.5000, 43.2500, 1.0194, -2.6326, 0.9006],
                             [24.5000, 13.2500, -18.0000, -0.4751, 0.9006, -1.4449]],
                            [[0, 0, 0, 1.0000, 0, 0],
                             [0, 0, 0, 0, 1.0000, 0],
                             [0, 0, 0, 0, 0, 1.0000],
                             [-73.5000, 59.0000, -44.5000, -0.4849, -0.0495, 0.4751],
                             [49.0000, 26.5000, 16.7500, -0.0495, -0.4948, -0.0495],
                             [-24.5000, -13.2500, -44.5000, 0.4751, -0.0495, -1.4449]]]
        
        uVec = [4,2,5]
        vVec = [3, 2, 2]

        iHfunc = Function("ihfunc", [self.engine.q], [pinv(self.H)], ['q'], ['iH'])

        print("Testing A matrix")
        for testIdx, (initialPosition, correctA) in enumerate(zip(self.initialPositions, correctMatricesA)):
            iH = iHfunc(initialPosition)
            npVal = np.array(DM(handlerLinearizedModel.getA(initialPosition,vVec,uVec,iH)))
            correctNpVal = np.array(correctA)
            np.testing.assert_allclose(np.squeeze(npVal.T), correctNpVal, rtol=7)
            print("A matrix test {}/{} complete".format(testIdx + 1, len(self.initialPositions)))
        print()
