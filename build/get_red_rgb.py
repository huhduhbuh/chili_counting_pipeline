import open3d as o3d
import numpy as np
import sys
import json

def extract_red_points(pcd, red_threshold=150, green_threshold=130, blue_threshold=130):
    """
    Extract points that are predominantly red from a point cloud.

    Parameters:
    - pcd: open3d.geometry.PointCloud
    - red_threshold: minimum R value (0-255)
    - green_threshold: maximum G value (0-255)
    - blue_threshold: maximum B value (0-255)

    Returns:
    - red_pcd: open3d.geometry.PointCloud containing only red points
    """
    # Convert colors to 0-255 range
    colors = np.asarray(pcd.colors) * 255  # Open3D stores colors in 0-1
    points = np.asarray(pcd.points)


    #print("R min/max:", colors[:,0].min(), colors[:,0].max())
    #print("G min/max:", colors[:,1].min(), colors[:,1].max())
    #print("B min/max:", colors[:,2].min(), colors[:,2].max())

    # Boolean mask for red points
    mask = (colors[:, 0] > red_threshold) & \
           (colors[:, 1] < green_threshold) & \
           (colors[:, 2] < blue_threshold)

    red_points = points[mask]
    red_colors = colors[mask] / 255.0  # back to 0-1 for Open3D

    # Create new point cloud
    red_pcd = o3d.geometry.PointCloud()
    red_pcd.points = o3d.utility.Vector3dVector(red_points)
    red_pcd.colors = o3d.utility.Vector3dVector(red_colors)

    return red_pcd

if __name__ == "__main__":

    input_file = sys.argv[1]
    config_path = sys.argv[2]

    with open(config_path) as f:
        config = json.load(f)

    # Load your point cloud (.ply or .pcd)
    pcd = o3d.io.read_point_cloud(input_file)

    # Extract red points
    red_only = extract_red_points(pcd, red_threshold=int(config['color_space']['rgb']['r']), green_threshold=int(config['color_space']['rgb']['g']), blue_threshold=int(config['color_space']['rgb']['b']))

    # Save red points
    o3d.io.write_point_cloud("red_rgb.pcd", red_only)  

    # Visualize
    #o3d.visualization.draw_geometries([red_only], window_name="Red Points")

    print('red_rgb.pcd')
