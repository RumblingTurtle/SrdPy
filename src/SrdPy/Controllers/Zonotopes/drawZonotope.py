from . import getZonotopeVertices
import numpy as np
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
def drawZonotope(G, h, faceColor=[0.8,0.2,0.2],faceAlpha=0.3):

        vertices = getZonotopeVertices(G) + h;

        indices_convhull = ConvexHull(vertices.T).vertices
        vertices = vertices[:, indices_convhull]

        plt.fill(vertices[1].T, vertices[2].T)
        plt.show()