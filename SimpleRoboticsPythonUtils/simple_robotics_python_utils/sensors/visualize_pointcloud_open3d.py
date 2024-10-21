import open3d as o3d
import numpy as np

# Define the file name
PCD_FILE_NAME = "/home/rico/dream_cartographer_ws/src/dream_cartographer/rgbd_slam_rico/data/rgbd_rico_slam_output.pcd"

# Load and visualize the PCD file
pcd_load = o3d.io.read_point_cloud(PCD_FILE_NAME)
num_points = len(pcd_load.points)
print(f"Number of points to visualize: {num_points}")
# Create a visualizer
vis = o3d.visualization.Visualizer()
vis.create_window()

# Add the point cloud to the visualizer
vis.add_geometry(pcd_load)

# Get the render option and set the point size
render_option = vis.get_render_option()
render_option.point_size = 1.5  # Adjust this value as needed

# Visualize the point cloud
vis.run()
vis.destroy_window()
