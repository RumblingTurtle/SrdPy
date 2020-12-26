import unittest
from SrdPy.Tests import *
from SrdPy.SymbolicUtils import *
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

    def generateJSIM(self):
        deriveJacobiansForlinkArray(self.engine)
        self.h = deriveJSIM(self.engine)
        self.hfunc = Function("hfunc",[self.engine.q], [self.h],
                                            ['q'], ['h'])

    def testJSIM(self):

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

            npVal = np.array(DM(self.hfunc(initialPosition)))
            correctNpVal = np.array(correctH)
            np.testing.assert_allclose(npVal,correctNpVal,rtol=9)
            print("JSIM test {}/{} complete".format(testIdx+1,len(correctHMatrices)))