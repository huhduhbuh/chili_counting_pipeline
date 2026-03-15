import open3d as o3d
import sys
import numpy as np

# Usage: python overlay_cloud.py original.ply segmented.ply overlay=true gray=true

og_path = sys.argv[1]
subset_path = sys.argv[2]
overlay = sys.argv[3].lower() == "true"
gray = sys.argv[4].lower() == "true"

flip = np.array([
    [1, 0, 0, 0],
    [0, -1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])


if overlay == True:
    pcd_og = o3d.io.read_point_cloud(og_path)
    pcd_og.transform(flip)

pcd_segmented = o3d.io.read_point_cloud(subset_path)
pcd_segmented.transform(flip)

# Color them
if overlay == True and gray == True:
    pcd_og.paint_uniform_color([0.7, 0.7, 0.7])   # gray background
#pcd_segmented.paint_uniform_color([1, 0, 0])      # red highlighted subset

# Visualize together
if overlay == True:
    o3d.visualization.draw_geometries([pcd_og, pcd_segmented])
else:
    o3d.visualization.draw_geometries([pcd_segmented])
