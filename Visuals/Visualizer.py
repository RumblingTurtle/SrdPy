import numpy as np
import meshcat.geometry as g
import meshcat.transformations as tf
from meshcat.animation import Animation
from SrdPy.Visuals import getCylinderTransform

class Visualizer:
    def __init__(self, jupyter = True):
        self.jupyter = jupyter

    def show(self, chain,showMeshes=False):
        if self.jupyter:
            from meshcat.jupyter import JupyterVisualizer
            vis = JupyterVisualizer()
        else:
            from meshcat import Visualizer
            vis = Visualizer().open()

        if showMeshes:
            for i,link in enumerate(chain.linkArray):
                if link.meshObj == None:
                    continue
                boxVis = vis["link"+str(i)]

                boxVis.set_object(link.meshObj,g.MeshLambertMaterial(
                             color=0xff22dd,
                             reflectivity=0.8))
                rotationMatrix = np.pad(link.absoluteOrientation, [(0, 1), (0, 1)], mode='constant')
                rotationMatrix[-1][-1] = 1
                boxVis.set_transform(tf.translation_matrix(link.absoluteBase)@rotationMatrix)

        else:
            vertices = chain.get_vertex_coords()

            for i in range(int(vertices.shape[0]/2)):
                p1 = vertices[2*i]
                p2 = vertices[2*i+1]

                cylinder_transform = getCylinderTransform(p1,p2)
                boxVis = vis["link"+str(i)]
                boxVis.set_object(g.Cylinder(1, 0.01))
                boxVis.set_transform(cylinder_transform)



    def animate(self,chain,states,framerate = 5):
        if self.jupyter:
            from meshcat.jupyter import JupyterVisualizer
            vis = JupyterVisualizer()
        else:
            from meshcat import Visualizer
            vis = Visualizer().open()
        anim = Animation()

        vertices = chain.get_vertex_coords()

        for i in range(int(vertices.shape[0]/2)):
            p1 = vertices[2*i]
            p2 = vertices[2*i+1]

            cylinder_transform = self.get_cylinder_transform(p1, p2)
            boxVis = vis["link"+str(i)]
            boxVis.set_object(g.Cylinder(1, 0.01))
            boxVis.set_transform(cylinder_transform)

        for i in range(len(states)):
            chain.update(states[i])
            with anim.at_frame(vis, framerate*i) as frame:
                vertices = chain.get_vertex_coords()

                for i in range(int(vertices.shape[0] / 2)):

                    p1 = vertices[2 * i]
                    p2 = vertices[2 * i + 1]

                    cylinder_transform = self.get_cylinder_transform(p1, p2)
                    boxVis = frame["link"+str(i)]
                    boxVis.set_transform(cylinder_transform)

        vis.set_animation(anim)