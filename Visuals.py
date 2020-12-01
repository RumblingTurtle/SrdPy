import numpy as np
from meshcat.jupyter import JupyterVisualizer
import meshcat.geometry as g
import meshcat.transformations as tf
from SrdMath import rotation_transform

class SrdVisualizer:
    def __init__(self):
        pass

    def show(self, chain, jupyter = True):
        if jupyter:
            vis = JupyterVisualizer()

            vertices = np.array([[0, 0, 0]])
            for i, link in enumerate(chain.linkArray):
                if link.name != "Ground":
                    vertices = np.vstack((vertices, link.absoluteBase.tolist()))

            cylinder_up_vector = [0, 1, 0]
            chain.linkArray[0].absoluteBase = np.array([0, 0, 0])
            for i in range(1, len(chain.linkArray) - 1):
                linkA = chain.linkArray[i]
                linkB = chain.linkArray[i + 1]

                cylinder_direction = linkB.absoluteBase - linkA.absoluteBase

                rotation_matrix = rotation_transform(cylinder_up_vector,
                                                     cylinder_direction / np.linalg.norm(cylinder_direction))

                boxVis = vis[linkA.name]
                boxVis.set_object(g.Cylinder(np.linalg.norm(cylinder_direction), 0.01))
                boxVis.set_transform(
                    tf.translation_matrix(linkA.absoluteBase + 0.5 * cylinder_direction).dot(
                        np.array(rotation_matrix)))