from SrdPy.plotGeneric import plotGeneric
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


    '''
    c1s = np.squeeze(blank_chain.links["RL_calf"].absoluteFollower)
    c2s = np.squeeze(blank_chain.links["RR_calf"].absoluteFollower)
    c3s = np.squeeze(blank_chain.links["FL_calf"].absoluteFollower)
    c4s = np.squeeze(blank_chain.links["FR_calf"].absoluteFollower)
    com1=blank_chain.getCoM()
    tasks = np.hstack((com1,c1s, c2s,c3s,c4s))
    
    
    initialPosition[1] = initialPosition[0]-0.2

    initialPosition[14] = initialPosition[14]+0.3
    initialPosition[8] = initialPosition[8]-0.3

    initialPosition[10] = initialPosition[14]+0.3
    initialPosition[16] = initialPosition[8]-0.3

    initialPosition[17] = initialPosition[17]+0.6
    initialPosition[11] = initialPosition[11]-0.6
    
    blank_chain.update(initialPosition)

    c1e = np.squeeze(blank_chain.links["RL_calf"].absoluteFollower)
    c2e = np.squeeze(blank_chain.links["RR_calf"].absoluteFollower)
    c3e = np.squeeze(blank_chain.links["FL_calf"].absoluteFollower)
    c4e = np.squeeze(blank_chain.links["FR_calf"].absoluteFollower)
    om1=blank_chain.getCoM()
    taske = np.hstack((om1,c1e, c2e, c3e,c4e))

    print(taske-tasks) 
    vis = Visualizer()
    vis.show(blank_chain,True)
    input()
    '''
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