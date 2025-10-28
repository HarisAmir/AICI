#!/usr/bin/env python3
"""
Multi-Frame RGB-D Point Cloud with Proper Coordinate Transformations
Transforms points from camera coordinates to ego vehicle / world coordinates
"""

from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
import numpy as np
import cv2
import open3d as o3d
from scipy.spatial.transform import Rotation as R
# -----------------------------------------------------------
# Configuration
# -----------------------------------------------------------
bag_path = Path("office/rosbag2_2025_10_20-16_09_39")
typestore = get_typestore(Stores.ROS2_HUMBLE)

rgb_topic = "/zed/zed_node/rgb/image_rect_color/compressed"
depth_topic = "/zed/zed_node/depth/depth_registered/compressedDepth"
odom_topic = "/zed/zed_node/odom"  # ZED-specific odometry
odom_root_topic = "/odom"  # General odometry
tf_static_topic = "/tf_static"  # Static transforms
tf_topic = "/tf"  # Dynamic transforms

# Camera intrinsics (from CameraInfo topic)
fx, fy = 524.73699951, 524.73699951
cx, cy = 649.56481934, 368.82150269

def load_tf_transforms(bag_path):
    """
    Load TF transforms from the rosbag
    Returns a dict of transforms: {child_frame: {'parent': parent, 'translation': [...], 'rotation': R}}
    """
    transforms = {}
    
    with AnyReader([bag_path], default_typestore=typestore) as reader:
        # Load static transforms
        tf_static_count = 0
        for conn, ts, raw in reader.messages(connections=reader.connections):
            if conn.topic == "/tf_static":
                msg = reader.deserialize(raw, conn.msgtype)
                
                for transform in msg.transforms:
                    # Convert quaternion to rotation matrix
                    r = R.from_quat([
                        transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                        transform.transform.rotation.w
                    ])
                    
                    transforms[transform.child_frame_id] = {
                        'parent': transform.header.frame_id,
                        'translation': np.array([
                            transform.transform.translation.x,
                            transform.transform.translation.y,
                            transform.transform.translation.z
                        ]),
                        'rotation': r.as_matrix()
                    }
                
                tf_static_count += 1
                if tf_static_count >= 2:
                    break
    
    return transforms

def chain_transforms(tf_transforms, source_frame, target_frame):
    """
    Chain TF transforms to get transformation from source to target frame
    Returns: (rotation_matrix, translation_vector)
    """
    # Try to find a path from source to target
    current_frame = source_frame
    total_rotation = np.eye(3)
    total_translation = np.zeros(3)
    
    max_depth = 10  # Prevent infinite loops
    depth = 0
    
    while current_frame != target_frame and depth < max_depth:
        if current_frame in tf_transforms:
            transform = tf_transforms[current_frame]
            
            # Compose transformations
            total_translation = total_translation + total_rotation @ transform['translation']
            total_rotation = transform['rotation'] @ total_rotation
            
            current_frame = transform['parent']
            depth += 1
        else:
            return None, None
    
    if current_frame == target_frame:
        return total_rotation, total_translation
    
    return None, None

def get_robot_pose_from_odom(odom_msg):
    """
    Extract robot pose from odometry message
    Returns: (position, rotation_matrix)
    """
    pos = odom_msg.pose.pose.position
    orientation = odom_msg.pose.pose.orientation
    
    # Convert quaternion to rotation matrix
    r = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
    rotation_matrix = r.as_matrix()
    
    position = np.array([pos.x, pos.y, pos.z])
    
    return position, rotation_matrix

def apply_robot_pose_transform(points_base, robot_position, robot_rotation):
    """
    Transform points from base_link frame to odom/world frame
    Using robot's current pose from odometry
    """
    # Apply rotation then translation (SE3 transform)
    points_world = points_base @ robot_rotation.T
    points_world = points_world + robot_position
    
    return points_world

def apply_camera_to_base_transform(points_camera, tf_transforms=None):
    """
    Transform points from camera optical frame to base_link frame
    Chains TF transforms: camera optical → camera_link → base_link
    """
    if tf_transforms is None:
        # Fallback to default transformation (90 deg rotation around X)
        print("No TF transforms found, using default transformation")
        rot_matrix = R.from_euler('x', 90, degrees=True).as_matrix()
        points_base = points_camera @ rot_matrix.T
        return points_base
    
    # Chain transforms from camera optical to base_link
    # Path: zed_left_camera_optical_frame → zed_left_camera_frame → zed_camera_center → zed_camera_link → base_link
    
    rotation, translation = chain_transforms(tf_transforms, 
                                            'zed_left_camera_optical_frame', 
                                            'base_link')
    
    if rotation is not None and translation is not None:
        # Apply chained transformation
        points_base = points_camera @ rotation.T
        points_base = points_base + translation
        return points_base
    
    # If chaining fails, use default
    print("Chaining failed, using default transformation")
    rot_matrix = R.from_euler('x', 90, degrees=True).as_matrix()
    points_base = points_camera @ rot_matrix.T
    return points_base

def main():
    
    # Load TF transforms from rosbag
    print("\n📋 Loading TF transforms from rosbag...")
    tf_transforms = load_tf_transforms(bag_path)
        
    # Storage for accumulated data
    all_points = []
    all_colors = []
    
    print("\n🔍 Reading messages from rosbag...")
    
    with AnyReader([bag_path], default_typestore=typestore) as reader:
        # First collect all RGB, depth, and odometry data separately
        print("📋 Collecting RGB, depth, and odometry data...")
        
        rgb_data = {}
        depth_data = {}
        odom_data = {}  # NEW: Store odometry for each frame
        
        for conn, ts, raw in reader.messages(connections=reader.connections):
            if conn.topic == rgb_topic:
                msg = reader.deserialize(raw, conn.msgtype)
                rgb_arr = np.frombuffer(msg.data, np.uint8)
                rgb_img = cv2.imdecode(rgb_arr, cv2.IMREAD_COLOR)
                if rgb_img is not None:
                    rgb_data[ts] = rgb_img
            
            elif conn.topic == depth_topic:
                msg = reader.deserialize(raw, conn.msgtype)
                data = np.frombuffer(msg.data, np.uint8)
                header_idx = bytes(data).find(b'\x89PNG\r\n\x1a\n')
                if header_idx != -1:
                    png_data = data[header_idx:]
                    depth_img = cv2.imdecode(png_data, cv2.IMREAD_UNCHANGED)
                    if depth_img is not None:
                        depth_data[ts] = depth_img
            
            elif conn.topic == odom_root_topic:  # NEW: Collect odometry
                msg = reader.deserialize(raw, conn.msgtype)
                odom_data[ts] = msg
        
        print(f"✅ Collected: {len(rgb_data)} RGB frames, {len(depth_data)} depth frames, {len(odom_data)} odom messages")
        print(f"   (Difference: {len(rgb_data) - len(depth_data)} frames)")
        
        # Now process synchronized pairs
        print("\n📷 Processing synchronized RGB-D pairs...")
        
        rgb_timestamps = sorted(rgb_data.keys())
        depth_timestamps = sorted(depth_data.keys())
        
        tolerance = 100000000  # 100ms tolerance
        # tolerance = 1000000000000000
        processed_frames = 0
        
        # Use fewer frames for cleaner output
        start_idx = 0
        end_idx = 500  # Process fewer frames for cleaner visualization
        
        print(f"📷 Processing frames {start_idx} to {end_idx-1}...")
        
        for i in range(start_idx, end_idx):
            rgb_ts = rgb_timestamps[i]
            rgb_img = rgb_data[rgb_ts]
            
            # Find closest depth frame
            depth_ts = min(depth_timestamps, key=lambda x: abs(x - rgb_ts))
            time_diff = abs(depth_ts - rgb_ts)
            
            if time_diff > tolerance:
                print(f"  ⚠️ Skipping RGB frame {i+1} - time diff too large: {time_diff/1000000:.1f}ms")
                continue
            
            depth_img = depth_data[depth_ts]
            
            # Convert to point cloud (depth confirmed to be in millimeters)
            depth_m = depth_img.astype(np.float32) / 1000.0  # Convert mm to meters
            
            # Filter valid depth values
            mask = (depth_m > 0.1) & (depth_m < 10.0)
            depth_m[~mask] = np.nan
            
            height, width = depth_m.shape
            u, v = np.meshgrid(np.arange(width), np.arange(height))
            X = (u - cx) * depth_m / fx
            Y = (v - cy) * depth_m / fy
            Z = depth_m
            
            points = np.stack((X, Y, Z), axis=-1).reshape(-1, 3)
            colors = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2RGB).reshape(-1, 3) / 255.0
            
            mask_flat = np.isfinite(points[:, 2])
            points_camera = points[mask_flat]
            colors = colors[mask_flat]
            
            if len(points_camera) > 0:
                # Step 1: Transform points from camera optical to base_link coordinates
                points_base = apply_camera_to_base_transform(points_camera, tf_transforms)
                
                # Step 2: Get robot pose from odometry
                # Find closest odometry message to this frame's timestamp
                odom_ts = min(odom_data.keys(), key=lambda x: abs(x - rgb_ts))
                odom_msg = odom_data[odom_ts]
                
                # Extract robot pose
                robot_pos, robot_rot = get_robot_pose_from_odom(odom_msg)
                
                # Step 3: Transform points from base_link to odom/world frame
                points_world = apply_robot_pose_transform(points_base, robot_pos, robot_rot)
                
                # Use world coordinate points
                points = points_world
                # Downsample each frame to prevent memory issues
                max_points_per_frame = 5000  # More points per frame for better quality
                if len(points) > max_points_per_frame:
                    indices = np.random.choice(len(points), max_points_per_frame, replace=False)
                    points = points[indices]
                    colors = colors[indices]
                
                all_points.append(points)
                all_colors.append(colors)
                processed_frames += 1
                
                # Progress reporting every 50 frames
                if processed_frames % 50 == 0:
                    print(f"✅ Processed {processed_frames} frames... ({len(points)} points)")
        
        print(f"✅ Processed {processed_frames} synchronized frames")
    
    # Combine all point clouds
    print("\n🔗 Combining point clouds...")
    
    if all_points:
        combined_points = np.vstack(all_points)
        combined_colors = np.vstack(all_colors)
        
        print(f"✅ Combined point cloud: {len(combined_points)} points")
        print(f"   (from {len(all_points)} frames)")
         
        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(combined_points)
        pcd.colors = o3d.utility.Vector3dVector(combined_colors)
        
        # Save PLY file (already sparse from per-frame downsampling)
        output_file = "results/office_pcd_500.ply"
        o3d.io.write_point_cloud(output_file, pcd)
        print(f"💾 Saved sparse point cloud → '{output_file}' ({len(combined_points)} points)")
         
    else:
        print("❌ No point cloud data to process!")

if __name__ == "__main__":
    main()
