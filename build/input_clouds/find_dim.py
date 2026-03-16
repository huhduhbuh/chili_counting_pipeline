import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("final_diamond.ply")

# Get axis-aligned bounding box
bbox = pcd.get_axis_aligned_bounding_box()

# Min and max coordinates
min_bound = bbox.get_min_bound()
max_bound = bbox.get_max_bound()

# Dimensions
dimensions = max_bound - min_bound

print("Min:", min_bound)
print("Max:", max_bound)
print("Dimensions (x, y, z):", dimensions)