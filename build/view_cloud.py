import open3d as o3d
import sys

# Get file path from command line
file_path = sys.argv[1]

# Load point cloud
pcd = o3d.io.read_point_cloud(file_path)

# Show viewer
o3d.visualization.draw_geometries([pcd])