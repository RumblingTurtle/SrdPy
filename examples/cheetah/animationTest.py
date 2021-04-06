from SrdPy.plotGeneric import plotGeneric
from SrdPy.URDFUtils import getLinkArrayFromURDF

from SrdPy.Visuals import Visualizer
from copy import deepcopy

from SrdPy.TableGenerators import *
from SrdPy import Chain
import numpy as np
import os

cheetahLinks = getLinkArrayFromURDF(os.path.abspath("./SrdPy/examples/cheetah/cheetah/urdf/cheetah.urdf"),True)
cheetahChain = Chain(cheetahLinks)
remap = [
'trunk',
'FL_hip',
'FL_thigh',
'FL_calf',
'FL_foot',
'FR_hip',
'FR_thigh',
'FR_calf',
'FR_foot',
'RL_hip',
'RL_thigh',
'RL_calf',
'RL_foot',
'RR_hip',
'RR_thigh',
'RR_calf',
'RR_foot'
]

cheetahChain.remapGenCoords(remap)
print(cheetahChain)
initialPosition = np.zeros(18)
blank_chain = deepcopy(cheetahChain)
blank_chain.update(initialPosition)

with open('anim_array.npy', 'rb') as f:
    q = np.load(f)

blank_chain.update(q[0])
q[np.abs(q) < 1e-4] = 0
idx = np.argwhere(np.all(q[..., :] == 0, axis=0))
print(idx)
plotGeneric(np.arange(q.shape[0]),q,plot=True)
vis = Visualizer()
vis.animate(blank_chain,q,framerate=0.1,showMeshes=True)

input() 