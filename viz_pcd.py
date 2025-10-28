#!/usr/bin/env python3
"""
Simple Point Cloud Visualizer
Loads and visualizes the multi-frame RGB-D point cloud
"""

import open3d as o3d
import numpy as np
import argparse
from pathlib import Path

def visualize_pointcloud(ply_file):
    """Load and visualize point cloud"""
    
    print(f"🔍 Loading point cloud: {ply_file}")
    
    # Check if file exists
    if not Path(ply_file).exists():
        print(f"❌ File not found: {ply_file}")
        return
    
    # Load point cloud
    pcd = o3d.io.read_point_cloud(ply_file)
    
    # Get point cloud bounds
    points = np.asarray(pcd.points)
  
    # Create visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Multi-Frame RGB-D Point Cloud", width=1200, height=800)
    
    # Add point cloud
    vis.add_geometry(pcd)
    
    # Configure rendering options
    opt = vis.get_render_option()
    opt.point_size = 3.0  # Thicker points
    opt.background_color = np.asarray([1, 1, 1])  # White background
    opt.show_coordinate_frame = True
    
    # Run visualization
    vis.run()
    vis.destroy_window()
    
    print("✅ Visualization complete!")

def main():
    # Set up argument parser
    parser = argparse.ArgumentParser()
      
    parser.add_argument(
        '-f', '--file',
        dest='ply_file',
        help='Path to PLY file'
    )
    
    args = parser.parse_args()
    
    ply_file = args.ply_file 
 
    visualize_pointcloud(ply_file)

if __name__ == "__main__":
    main()
