from SrdPy.Tests.ThreeLink import *
from SrdPy.Tests.Cheetah import *

class TestSuite():
    def __init__(self):
        pass
    
    def runTests(self):
        self.runCheetahTests()
        #self.runThreeLinkTests()

    def runCheetahTests(self):
        cheetahTester = CheetahTests()
        cheetahTester.testIKModelHandler()

    def runThreeLinkTests(self):
       threeLinkTester = ThreeLinkTests()
       threeLinkTester.testJSIM()
       threeLinkTester.testModelVectorC()
       threeLinkTester.testLinearizationMatrixA()
