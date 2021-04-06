import imp
from SrdPy import Chain
from SrdPy.URDFUtils import getLinkArrayFromURDF
import os

def getCheetahChain():
    cheetahLinks = getLinkArrayFromURDF(os.path.abspath("./SrdPy/examples/cheetah/cheetah/urdf/cheetah.urdf"),True)
    cheetahChain = Chain(cheetahLinks)
    return cheetahChain
    