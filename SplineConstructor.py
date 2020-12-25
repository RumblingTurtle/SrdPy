from SrdPy.SrdSpline import SrdSpline
import numpy as np

class SplineNode():
    def __init__(self):
        self.V1 = []
        self.V2 = []
        self.V3 = []
        self.time = 0

class SplineConstructor():
    def __init__(self):

        self.nodeTimes = []
        self.nodeValues = []
        self.splineArray = [[],[],[]]
        self.numberOfSplines = 0
        self.outOfBoundsBehaviour = "Loop"
        self.outOfBoundsValue = 0

    def generateSplines(self, nodeTimes, zeroOrderDerivatives, firstOrderDerivatives, secondOrderDerivatives):
        zeroOrderShape = (len(zeroOrderDerivatives),len(zeroOrderDerivatives[0]))
        firstOrderShape = (len(firstOrderDerivatives), len(firstOrderDerivatives[0]))
        secondOrderShape = (len(secondOrderDerivatives), len(secondOrderDerivatives[0]))
        nodeTimesLength = len(nodeTimes)

        if zeroOrderShape[1]!=nodeTimesLength:
            raise RuntimeError("Inconsistent shapes: zeroOrderDerivatives is "+str(zeroOrderShape[1])
                        +" and nodeTimes shape is "+str(nodeTimesLength))

        if firstOrderShape[1]!=nodeTimesLength:
            raise RuntimeError("Inconsistent shapes: firstOrderDerivatives is "+str(firstOrderShape[1])
                        +" and nodeTimes shape is "+str(nodeTimesLength))

        if secondOrderShape[1]!=nodeTimes.shape[0]:
            raise RuntimeError("Inconsistent shapes: secondOrderDerivatives is "+str(secondOrderShape[1])
                        +" and nodeTimes shape is "+str(nodeTimesLength))

        self.nodeTimes = nodeTimes
        self.zeroOrderDerivatives = zeroOrderDerivatives
        self.firstOrderDerivatives = firstOrderDerivatives
        self.secondOrderDerivatives = secondOrderDerivatives

        self.numberOfSplines = zeroOrderShape[0]

        for i in range(self.numberOfSplines):
            V1 = self.zeroOrderDerivatives[i]
            V2 = self.firstOrderDerivatives[i]
            V3 = self.secondOrderDerivatives[i]

            nodes = []
            for j in range(zeroOrderShape[1]):
                node = SplineNode()
                node.time = self.nodeTimes[j]
                node.V1 = V1[j]
                node.V2 = V2[j]
                node.V3 = V3[j]
                nodes.append(node)

            segments = []
            for j in range(len(nodes)-1):
                node1 = nodes[j]
                node2 = nodes[j+1]

                segment = [[],[],[]]
                
                segment[0].append(node1.time)
                segment[0].append(node2.time)

                segment[1].append(node1.V1)
                segment[1].append(node2.V1)

                segment[2].append(0)
                segment[2].append(0)

                segment[0].append(node1.time)
                segment[0].append(node2.time)

                segment[1].append(node1.V2)
                segment[1].append(node2.V2)

                segment[2].append(1)
                segment[2].append(1)

                segment[0].append(node1.time)
                segment[0].append(node2.time)

                segment[1].append(node1.V3)
                segment[1].append(node2.V3)

                segment[2].append(2)
                segment[2].append(2)

            segments.append(segment)

            nodeCount = len(nodes)
            timesForCurrentSpline = [0]*nodeCount
            for j in range(nodeCount):
                timesForCurrentSpline[j] = nodes[j].time
            new_spline = SrdSpline(segments,timesForCurrentSpline)
            self.splineArray[0].append(new_spline)
            self.splineArray[1].append(new_spline.derivativeSpline(1))
            self.splineArray[2].append(new_spline.derivativeSpline(2))

        return self.splineArray

    def evaluateQ(self, t):
        q = np.zeros((self.numberOfSplines, 1))
        for i in range(self.numberOfSplines):
            q[i] = self.splineArray[0][i].evaluate(t,0)
        return q

    def evaluateV(self, t):
        v = np.zeros((self.numberOfSplines, 1))
        for i in range(self.numberOfSplines):
            v[i] = self.splineArray[1][i].evaluate(t,0)
        return v

    def evaluateA(self, t):
        a = np.zeros((self.numberOfSplines, 1))
        for i in range(self.numberOfSplines):
            a[i] = self.splineArray[2][i].evaluate(t,0)
        return a

    def evaluateAll(self, t):
        return evaluateQ(t),evaluateV(t),evaluateA(t)
