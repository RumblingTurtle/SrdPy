import numpy as np
import meshcat.geometry as g
import meshcat.transformations as tf
from meshcat.animation import Animation
from SrdPy.Visuals import getCylinderTransform
import meshcat
import sys
import urdf_parser_py.urdf as primitives

class Visualizer:
    def __init__(self):
        pass

    def show(self, chain,showMeshes=False):
        if 'google.colab' in sys.modules:
            server_args = ['--ngrok_http_tunnel']
            # Start a single meshcat server instance to use for the remainder of this notebook.
            from meshcat.servers.zmqserver import start_zmq_server_as_subprocess
            proc, zmq_url, web_url = start_zmq_server_as_subprocess(server_args=server_args)
            vis = meshcat.Visualizer(zmq_url=zmq_url)
        else:
            vis = meshcat.Visualizer().open()

        if showMeshes:
            for i,link in enumerate(chain.linkArray):
                if link.meshObj == None:
                    print("No mesh: "+link.name)
                    continue
                boxVis = vis["link:"+link.name]

                boxVis.set_object(link.meshObj,g.MeshLambertMaterial(
                             color=0xffffff,
                             reflectivity=0.8))
                rotationMatrix = np.pad(link.absoluteOrientation, [(0, 1), (0, 1)], mode='constant')
                rotationMatrix[-1][-1] = 1
                boxVis.set_transform(tf.translation_matrix(link.absoluteBase)@rotationMatrix)

        else:
                    

            for i,link in enumerate(chain.linkArray):
                
                boxVis = vis["link:"+link.name]
                if link.primitiveObj != None:
                    if isinstance(link.primitiveObj,primitives.Box):
                        box = meshcat.geometry.Box(link.primitiveObj.size)
                        boxVis.set_object(box)
                    if isinstance(link.primitiveObj,primitives.Cylinder):
                        cylinder = meshcat.geometry.Cylinder(link.primitiveObj.length,link.primitiveObj.radius)
                        boxVis.set_object(cylinder)
                    if isinstance(link.primitiveObj,primitives.Sphere):
                        sphere = meshcat.geometry.Sphere(link.primitiveObj.radius)
                        boxVis.set_object(cylinder)
                    rotationMatrix = np.pad(link.absoluteOrientation, [(0, 1), (0, 1)], mode='constant')
                    rotationMatrix[-1][-1] = 1
                    boxVis.set_transform(tf.translation_matrix(link.absoluteBase)@rotationMatrix)
            
            boxVis = vis["skeleton"]
            boxVis.set_object(g.Line(g.PointsGeometry(chain.get_vertex_coords().T)))
            

    def animate(self,chain,states,framerate = 5,showMeshes=False):
        if 'google.colab' in sys.modules:
            server_args = ['--ngrok_http_tunnel']
            # Start a single meshcat server instance to use for the remainder of this notebook.
            from meshcat.servers.zmqserver import start_zmq_server_as_subprocess
            proc, zmq_url, web_url = start_zmq_server_as_subprocess(server_args=server_args)
            vis = meshcat.Visualizer(zmq_url=zmq_url)
        else:
            vis = meshcat.Visualizer().open()

        anim = Animation()

        vertices = chain.get_vertex_coords()

        if showMeshes:
            for i,link in enumerate(chain.linkArray):
                if link.meshObj == None:
                    print("No mesh: "+link.name)
                    continue
                boxVis = vis["link:"+link.name]

                boxVis.set_object(link.meshObj,g.MeshLambertMaterial(
                             color=0xffffff,
                             reflectivity=0.8))
                rotationMatrix = np.pad(link.absoluteOrientation, [(0, 1), (0, 1)], mode='constant')
                rotationMatrix[-1][-1] = 1
                boxVis.set_transform(tf.translation_matrix(link.absoluteBase)@rotationMatrix)

            for i in range(len(states)):
                chain.update(states[i])
                with anim.at_frame(vis, framerate*i) as frame:
                        for i,link in enumerate(chain.linkArray):
                            if link.meshObj == None:
                                continue

                            boxVis = frame["link:"+link.name]
                            rotationMatrix = np.pad(link.absoluteOrientation, [(0, 1), (0, 1)], mode='constant')
                            rotationMatrix[-1][-1] = 1
                            boxVis.set_transform(tf.translation_matrix(link.absoluteBase)@rotationMatrix)

        else:
            for i in range(int(vertices.shape[0]/2)):
                p1 = vertices[2*i]
                p2 = vertices[2*i+1]

                cylinder_transform = getCylinderTransform(p1, p2)
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

                        cylinder_transform = getCylinderTransform(p1, p2)
                        boxVis = frame["link"+str(i)]
                        boxVis.set_transform(cylinder_transform)

        vis.set_animation(anim)