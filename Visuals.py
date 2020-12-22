import numpy as np
import meshcat.geometry as g
import meshcat.transformations as tf
from SrdPy.SrdMath import rotation_transform
from meshcat.animation import Animation

class SrdVisualizer:
    def __init__(self, jupyter = True):
        self.jupyter = jupyter

    def show(self, chain):
        if self.jupyter:
            from meshcat.jupyter import JupyterVisualizer
            vis = JupyterVisualizer()
        else:
            from meshcat import Visualizer
            vis = Visualizer().open()

        vertices = chain.get_vertex_coords()

        for i in range(len(chain.linkArray) - 1):
            p1 = chain.linkArray[i].absoluteBase
            p2 = chain.linkArray[i + 1].absoluteBase

            cylinder_transform = self.get_cylinder_transform(p1,p2)
            boxVis = vis[chain.linkArray[i].name]
            boxVis.set_object(g.Cylinder(1, 0.01))
            boxVis.set_transform(cylinder_transform)

    @staticmethod
    def get_cylinder_transform(p1, p2):
        cylinder_up_vector = [0, 1, 0]
        cylinder_direction = p2 - p1

        if np.linalg.norm(cylinder_direction)==0:
            return np.zeros((4,4))

        rotation_matrix = rotation_transform(cylinder_up_vector,
                                             cylinder_direction / np.linalg.norm(cylinder_direction))
        scale_matrix = np.eye(4)
        scale_matrix[1,1] = np.linalg.norm(cylinder_direction)
        full_transform = tf.translation_matrix(p1 + 0.5 * cylinder_direction).dot(rotation_matrix.dot(scale_matrix))
        return np.array(full_transform)

    def animate(self,chain,states,framerate = 5):
        if self.jupyter:
            from meshcat.jupyter import JupyterVisualizer
            vis = JupyterVisualizer()
        else:
            from meshcat import Visualizer
            vis = Visualizer().open()
        anim = Animation()

        vertices = chain.get_vertex_coords()

        for i in range(len(chain.linkArray) - 1):
            p1 = chain.linkArray[i].absoluteBase
            p2 = chain.linkArray[i + 1].absoluteBase

            cylinder_transform = self.get_cylinder_transform(p1, p2)
            boxVis = vis[chain.linkArray[i].name]
            boxVis.set_object(g.Cylinder(1, 0.01))
            boxVis.set_transform(cylinder_transform)

        for i in range(len(states)):
            chain.update(states[i])
            with anim.at_frame(vis, framerate*i) as frame:
                vertices = chain.get_vertex_coords()

                for i in range(len(chain.linkArray) - 1):
                    p1 = chain.linkArray[i].absoluteBase
                    p2 = chain.linkArray[i + 1].absoluteBase

                    cylinder_transform = self.get_cylinder_transform(p1, p2)
                    boxVis = frame[chain.linkArray[i].name]
                    boxVis.set_transform(cylinder_transform)

        vis.set_animation(anim)