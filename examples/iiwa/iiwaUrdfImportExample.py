from SrdPy.URDFUtils import getLinkArrayFromURDF
from SrdPy import Chain
import numpy as np
from SrdPy.Visuals import Visualizer
import os
def iiwaImportExample():
    print(os.getcwd())
    iiwaLinks = getLinkArrayFromURDF(os.path.abspath("./SrdPy/examples/iiwa/iiwa14.urdf"),True)
    iiwaChain = Chain(iiwaLinks)
    iiwaChain.update(np.array([1]*7))
    vis = Visualizer()
    vis.show(iiwaChain,True)
    input()
