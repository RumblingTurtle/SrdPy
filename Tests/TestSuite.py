from SrdPy.Tests import ThreeLinkTests
class TestSuite():
    def __init__(self):
        pass

    def runTests(self):
       threeLinkTester = ThreeLinkTests()
       threeLinkTester.testJSIM()
       threeLinkTester.testModelVectorC()
       threeLinkTester.testLinearizationMatrixA()


suite = TestSuite()
suite.runTests()