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

rgb_255 = (rgb * 255).astype(np.uint8)
rgb_reshaped = rgb_255.reshape(-1, 1, 3)

lab = cv2.cvtColor(rgb_reshaped, cv2.COLOR_RGB2LAB)
lab = lab.reshape(-1, 3)

L = lab[:, 0]   # Lightness [0,255]
a = lab[:, 1]   # Green–Red (128 = neutral)
b = lab[:, 2]   # Blue–Yellow

# Strong red component
red_a = a >= int(config['color_space']['lab']['a'])     # increase for stricter red

# Optional: avoid dark noise
light_enough = L >= int(config['color_space']['lab']['l'])

# Optional: avoid yellow/orange confusion
not_yellow = b <= int(config['color_space']['lab']['b'])

red_mask = red_a & light_enough & not_yellow

points = np.asarray(pcd.points)

red_points = points[red_mask]
red_colors = rgb[red_mask]

red_pcd = o3d.geometry.PointCloud()
red_pcd.points = o3d.utility.Vector3dVector(red_points)
red_pcd.colors = o3d.utility.Vector3dVector(red_colors)

#o3d.visualization.draw([red_pcd])

o3d.io.write_point_cloud("red_lab.pcd", red_pcd)

print('red_lab.pcd', flush=True)
