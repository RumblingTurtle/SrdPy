import numpy as np


def rotationTransform(v1,v2):
  #https://math.stackexchange.com/a/3219491
  v = np.cross(v1, v2)
  u = v/np.linalg.norm(v)
  c = np.dot(v1, v2)
  if 1-c**2==0:
      return np.eye(4)
  h = (1 - c)/(1 - c**2)

  vx, vy, vz = v
  return np.array([[c + h*vx**2, h*vx*vy - vz, h*vx*vz + vy,0],
        [h*vx*vy+vz, c+h*vy**2, h*vy*vz-vx,0],
        [h*vx*vz - vy, h*vy*vz + vx, c+h*vz**2,0],
        [0,0,0,1]])