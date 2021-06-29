from SrdPy.Math.derivative import derivative
from SrdPy.Math.crossProductMatrix3D import crossProductMatrix3D
from SrdPy.Math.callForAllCombinations import callForAllCombinations
from SrdPy.Math.clampTo2pi import clampTo2pi
from SrdPy.Math.convexHull import convexHull
from SrdPy.Math.crossProductMatrix2D import crossProductMatrix2D
from SrdPy.Math.crossProductMatrix3D import crossProductMatrix3D
from SrdPy.Math.derivative import derivative
from SrdPy.Math.findPlaceInArray import findPlaceInArray
from SrdPy.Math.matrixDerivative import matrixDerivative
from SrdPy.Math.parallelizedSimplification import parallelizedSimplification
from SrdPy.Math.rotationTransform import rotationTransform
from SrdPy.Math.rotationMatrix2D import rotationMatrix2D
from SrdPy.Math.rotationMatrix3Dx import rotationMatrix3Dx
from SrdPy.Math.rotationMatrix3Dy import rotationMatrix3Dy
from SrdPy.Math.rotationMatrix3Dz import rotationMatrix3Dz
from SrdPy.Math.weightedPseudoinverse import weightedPseudoinverse
from SrdPy.Math.rpyToRotationMatrix import rpyToRotationMatrix
from SrdPy.Math.matrixJacobianTimesVector import matrixJacobianTimesVector
from SrdPy.Math.svd_suit import svd_suit
from SrdPy.Math.Tensor3MatrixProduct import Tensor3MatrixProduct

    
numberOfWorkers = 8
simplifyFunctions = True
useParallel = True
simplifySteps = 1
