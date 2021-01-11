from casadi import *
import numpy as np

class Spline:

    VALID_OUT_OF_BOUNDS_BEHAVIOURS = ["Loop","None","LastValue","Warning&LastValue","OutOfBoundsValue"]

    def __init__(self, segments = None, times = None):
        self.coefficients = [] #Coefficients of the spline
        self.outOfBoundsBehaviour = "Loop" #This property determines what the spline will do after
        self.outOfBoundsValue = []

        if times!=None:
            self.times = times #A column vector that determines for which segment of time will each polynomial be used.

        if segments==None:
            return

        t = SX.sym("t")
        coefficients = []
        for segment in segments:
            pointCount = len(segment[0])
            defaultMatrixLine = SX.zeros(pointCount)

            matrix = np.zeros((pointCount, pointCount))
            values = np.zeros((pointCount))
            for j in range(pointCount):
                defaultMatrixLine[j] = t**(pointCount-j-1)

            for j in range(pointCount):

                time = segment[0][j]
                values[j] = segment[1][j]
                derivativeOrder = segment[2][j]

                matrixLine = defaultMatrixLine

                for i in range(derivativeOrder):
                    matrixLine = jacobian(matrixLine,t)

                substituted_line = substitute(matrixLine,t,time)

                matrix[j] =  np.array(evalf(substituted_line)).reshape((pointCount))

            self.coefficients.append(np.linalg.solve(matrix,values))
        self.coefficients.append(self.coefficients[-1])


    def evaluate(self,time, order):
        if time > max(self.times):
            if self.outOfBoundsBehaviour == "OutOfBoundsValue":
                return self.outOfBoundsValue
            else:
                time = self.outOfBoundsFix(time)

        segmentCount = len(self.times)-1

        for i in range(segmentCount):
            if time>=self.times[i] and time<= self.times[i+1]:
                C = self.coefficients[i]

        if order > 0:
            C = np.polyder(C,order)

        return np.polyval(C,time)

    def derivativeSpline(self,order):
        derivativeSpline = Spline()
        derivativeSpline.times = np.copy(self.times)
        derivativeSpline.outOfBoundsBehaviour = self.outOfBoundsBehaviour

        segmentCount = len(self.coefficients)

        for i in range(segmentCount):
            C = np.polyder(self.coefficients[i],order)
            derivativeSpline.coefficients.append(C)

        return derivativeSpline

    def outOfBoundsFix(self, time):
        behaviour = self.outOfBoundsBehaviour
        if behaviour == "Loop":
            period = self.times[-1]
            numberOfPeriods = floor(time/period)
            return time-numberOfPeriods*period
        elif behaviour == "None":
            return time
        elif behaviour == "LastValue":
            return self.times[-1]
        elif behaviour == "Warning":
            warnings.warn("SrdSpline: The time is out of bounds")
            return self.times[-1]
        else:
            raise Warning("SrdSpline: Invalid OutOfBoundariesBehaviour type")
