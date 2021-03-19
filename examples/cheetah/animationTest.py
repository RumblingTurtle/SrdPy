from SrdPy.URDFUtils import getLinkArrayFromURDF

from SrdPy.Visuals import Visualizer
from copy import deepcopy

from SrdPy.TableGenerators import *
from SrdPy import Chain
import numpy as np
import os

def animationTest():
    cheetahLinks = getLinkArrayFromURDF(os.path.abspath("./SrdPy/examples/cheetah/cheetah/urdf/cheetah.urdf"),True)
    cheetahChain = Chain(cheetahLinks)
    print(cheetahChain)
    initialPosition = np.zeros(18)

    blank_chain = deepcopy(cheetahChain)
    blank_chain.update(initialPosition)
    with open('anim_array.npy', 'rb') as f:
        q = np.load(f)

    vis = Visualizer()
    vis.animate(blank_chain,q,framerate=0.1,showMeshes=True)