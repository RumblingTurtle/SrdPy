from SrdPy import SplineConstructor
class HandlerIK():
    def __init__(self):
        self.spline = None
        self.outOfBoundsValue = "LastValue"
        self.timeStart = 0
        self.timeExpiration = 0

    def getTask(self,t):
        return self.SC.evaluateQ(t)

    def getDerivative(self,t):
        return self.SC.evaluateV(t)

    def getTaskSecondDerivative(self,t):
        return self.SC.evaluateA(t)

def getIKtaskSplinesHandler(nodeTimes,zeroOrderDerivativeNodes,firstOrderDerivativeNodes,secondOrderDerivativeNodes):
    handler = HandlerIK()

    handler.SC = SplineConstructor()
    handler.spline = handler.SC\
        .generateSplines(nodeTimes,zeroOrderDerivativeNodes,firstOrderDerivativeNodes,secondOrderDerivativeNodes)

    handler.timeStart = nodeTimes[0]
    handler.timeExpiration = nodeTimes[-1]



