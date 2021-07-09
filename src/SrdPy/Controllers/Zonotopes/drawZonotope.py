from . import getZonotopeVertices
import numpy as np
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
def drawZonotope(G, h,faceAlpha=0.3):

        vertices = getZonotopeVertices(G) + h;
        if np.count_nonzero(vertices)==0:
                return
        indices_convhull = ConvexHull(vertices.T).vertices
        vertices = vertices[:, indices_convhull]
        ax = plt.gca()
        ax.fill(vertices[1].T, vertices[0].T,alpha=0.5,edgecolor='black')
        