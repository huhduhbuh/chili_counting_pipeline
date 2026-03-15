import open3d as o3d
import sys

# Usage: python overlay.py original.ply subset.ply

og_path = sys.argv[1]
subset_path = sys.argv[2]

# Load point clouds
pcd_og = o3d.io.read_point_cloud(og_path)
pcd_subset = o3d.io.read_point_cloud(subset_path)

# Color them
pcd_og.paint_uniform_color([0.7, 0.7, 0.7])   # gray background
#pcd_subset.paint_uniform_color([1, 0, 0])      # red highlighted subset

# Visualize together
o3d.visualization.draw_geometries([pcd_og, pcd_subset])