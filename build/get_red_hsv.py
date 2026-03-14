import open3d as o3d
import numpy as np
import cv2
import sys
import json

o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)


input_file = sys.argv[1]
config_path = sys.argv[2]

with open(config_path) as f:
    config = json.load(f)


# Load point cloud
pcd = o3d.io.read_point_cloud(input_file)

#o3d.visualization.draw([pcd])


# Open3D stores colors as float [0,1]
rgb = np.asarray(pcd.colors)          # shape (N, 3)
rgb_8u = (rgb * 255).astype(np.uint8)

# OpenCV expects BGR
bgr = rgb_8u[:, ::-1].reshape(-1, 1, 3)

# Convert RGB -> HSV
hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
hsv = hsv.reshape(-1, 3)

# Normalize HSV for visualization in Open3D
# H: [0,179] → [0,1]
# S,V: [0,255] → [0,1]
#hsv_vis = np.zeros_like(hsv, dtype=np.float32)
#hsv_vis[:, 0] = hsv[:, 0] / 179.0
#hsv_vis[:, 1] = hsv[:, 1] / 255.0
#hsv_vis[:, 2] = hsv[:, 2] / 255.0


h = hsv[:, 0]   # Hue   ∈ [0,179]
s = hsv[:, 1]   # Saturation
v = hsv[:, 2]   # Value

# Red range 1 (low hue)
lower_red_1 = (h <= int(config['color_space']['hsv']['h_lower']))

# Red range 2 (high hue)
lower_red_2 = (h >= int(config['color_space']['hsv']['h_upper']))

# Combine hue ranges
red_hue = lower_red_1 | lower_red_2

# Saturation & value filters
red_mask = (
    red_hue &
    (s >= int(config['color_space']['hsv']['s'])) &     # remove white/gray
    (v >= int(config['color_space']['hsv']['v']))       # remove dark noise
)

points = np.asarray(pcd.points)

red_points = points[red_mask]
red_colors = rgb[red_mask]

red_pcd = o3d.geometry.PointCloud()
red_pcd.points = o3d.utility.Vector3dVector(red_points)
red_pcd.colors = o3d.utility.Vector3dVector(red_colors)

#o3d.visualization.draw([red_pcd])

o3d.io.write_point_cloud('red_hsv.pcd', red_pcd)


#pcd.colors = o3d.utility.Vector3dVector(hsv_vis)

# Save and view
#o3d.io.write_point_cloud("hsv.ply", pcd)
#o3d.visualization.draw_geometries([pcd])

print('red_hsv.pcd', flush=True)



