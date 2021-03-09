from SrdPy import SplineConstructor
class IKtaskSplinesHandler():
    def __init__(self,nodeTimes,zeroOrderDerivativeNodes,firstOrderDerivativeNodes,secondOrderDerivativeNodes):
        self.SC = SplineConstructor()
        self.spline = self.SC\
            .generateSplines(nodeTimes,zeroOrderDerivativeNodes,firstOrderDerivativeNodes,secondOrderDerivativeNodes)

        self.timeStart = nodeTimes[0]
        self.timeExpiration = nodeTimes[-1]
        self.outOfBoundsValue = "LastValue"

    def getTask(self,t):
        return self.SC.evaluateQ(t)

    def getDerivative(self,t):
        return self.SC.evaluateV(t)

    def getTaskSecondDerivative(self,t):
        return self.SC.evaluateA(t)



