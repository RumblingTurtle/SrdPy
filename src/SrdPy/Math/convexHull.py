from scipy.spatial.qhull import ConvexHull


def convexHull(points):
    return points[ConvexHull(points).vertices]