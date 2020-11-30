import sys
import subprocess
import os
from meshcat.servers.zmqserver import match_web_url, match_zmq_url
import meshcat
import meshcat.geometry as g
import numpy as np

class SrdVisualizer:
    def __init__(self):
        args = [sys.executable, "-u", "-m", "meshcat.servers.zmqserver"]

        env = dict(os.environ)
        env["PYTHONPATH"] = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
        kwargs = {
            'stdout': subprocess.PIPE,
            'stderr': subprocess.PIPE,
            'env': env
        }
        # Use start_new_session if it's available. Without it, in jupyter the server
        # goes down when we cancel execution of any cell in the notebook.
        if sys.version_info.major >= 3:
            kwargs['start_new_session'] = True
        self.server_proc = subprocess.Popen(args, **kwargs)
        line = ""
        while "zmq_url" not in line:
            line = self.server_proc.stdout.readline().strip().decode("utf-8")
            if self.server_proc.poll() is not None:
                outs, errs = self.server_proc.communicate()
                print(outs.decode("utf-8"))
                print(errs.decode("utf-8"))
                raise RuntimeError(
                    "the meshcat server process exited prematurely with exit code " + str(self.server_proc.poll()))
        zmq_url = match_zmq_url(line)
        web_url = match_web_url(self.server_proc.stdout.readline().strip().decode("utf-8"))
        self.meshcatVis = meshcat.Visualizer(zmq_url=zmq_url)
        self.meshcatVis.open()

    def show(self, chain):
        vertices = np.array([])
        for link in chain.linkArray[1:]:
            print(link.absoluteBase)
            print(link.absoluteFollower)
            if vertices.shape[0] == 0:
                vertices = np.array([link.absoluteBase])
            else:
                vertices=np.vstack((vertices,[link.absoluteBase]))
            vertices=np.vstack((vertices,link.absoluteFollower))

        line_vertices = np.array(vertices).astype(np.float32)

        self.meshcatVis['lines_segments'].set_object(g.LineSegments(g.PointsGeometry(line_vertices)))



    def kill(self):
        self.server_proc.terminate()