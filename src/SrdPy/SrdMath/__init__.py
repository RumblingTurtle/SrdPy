from SrdPy.SrdMath.derivative import derivative
from SrdPy.SrdMath.crossProductMatrix3D import crossProductMatrix3D
from SrdPy.SrdMath.callForAllCombinations import callForAllCombinations
from SrdPy.SrdMath.clampTo2pi import clampTo2pi
from SrdPy.SrdMath.convexHull import convexHull
from SrdPy.SrdMath.crossProductMatrix2D import crossProductMatrix2D
from SrdPy.SrdMath.crossProductMatrix3D import crossProductMatrix3D
from SrdPy.SrdMath.derivative import derivative
from SrdPy.SrdMath.findPlaceInArray import findPlaceInArray
from SrdPy.SrdMath.matrixDerivative import matrixDerivative
from SrdPy.SrdMath.parallelizedSimplification import parallelizedSimplification
from SrdPy.SrdMath.rotationTransform import rotationTransform
from SrdPy.SrdMath.rotationMatrix2D import rotationMatrix2D
from SrdPy.SrdMath.rotationMatrix3Dx import rotationMatrix3Dx
from SrdPy.SrdMath.rotationMatrix3Dy import rotationMatrix3Dy
from SrdPy.SrdMath.rotationMatrix3Dz import rotationMatrix3Dz
from SrdPy.SrdMath.weightedPseudoinverse import weightedPseudoinverse
from SrdPy.SrdMath.rpyToRotationMatrix import rpyToRotationMatrix


numberOfWorkers = 8
simplifyFunctions = True
useParallel = True
simplifySteps = 1
