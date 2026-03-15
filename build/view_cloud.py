import open3d as o3d
import sys
import numpy as np

file_path = sys.argv[1]

pcd = o3d.io.read_point_cloud(file_path)

# Flip along Z axis
flip = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, -1, 0],
    [0, 0, 0, 1]
])

pcd.transform(flip)

o3d.visualization.draw_geometries([pcd])