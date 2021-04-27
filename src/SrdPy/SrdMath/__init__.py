from .crossProductMatrix3D import crossProductMatrix3D
from .callForAllCombinations import callForAllCombinations
from .clampTo2pi import clampTo2pi
from .convexHull import convexHull
from .crossProductMatrix2D import crossProductMatrix2D
from .crossProductMatrix3D import crossProductMatrix3D
from .findPlaceInArray import findPlaceInArray
from .matrixDerivative import matrixDerivative
from .parallelizedSimplification import parallelizedSimplification
from .rotationTransform import rotationTransform
from .rotationMatrix2D import rotationMatrix2D
from .rotationMatrix3Dx import *
from .rotationMatrix3Dy import rotationMatrix3Dy
from .rotationMatrix3Dz import rotationMatrix3Dz
from .weightedPseudoinverse import weightedPseudoinverse
from .rpyToRotationMatrix import rpyToRotationMatrix
from .TransformHandler3DX_rotation import TransformHandler3DX_rotation
from .TransformHandler3DY_rotation import TransformHandler3DY_rotation
from .TransformHandler3DZ_rotation import TransformHandler3DZ_rotation

numberOfWorkers = 8
simplifyFunctions = True
useParallel = True
simplifySteps = 1
