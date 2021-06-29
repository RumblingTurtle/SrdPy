import numpy as np
import meshcat.transformations as tf
from SrdPy.Math import rotationTransform
def getCylinderTransform(p1, p2):
    cylinder_up_vector = [0, 1, 0]
    cylinder_direction = p2 - p1

    if np.linalg.norm(cylinder_direction) == 0:
        return np.zeros((4, 4))

    rotation_matrix = rotationTransform(cylinder_up_vector,
                                         cylinder_direction / np.linalg.norm(cylinder_direction))
    scale_matrix = np.eye(4)
    scale_matrix[1, 1] = np.linalg.norm(cylinder_direction)
    full_transform = tf.translation_matrix(p1 + 0.5 * cylinder_direction).dot(rotation_matrix.dot(scale_matrix))
    return np.array(full_transform)
