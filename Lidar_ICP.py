import numpy as np
import open3d as o3d
import os
import glob
from tqdm import tqdm

def load_velo_scan(file_path):
    """Reads a KITTI .bin LiDAR scan and returns an Open3D PointCloud."""
    # KITTI LiDAR data is stored as float32 [x, y, z, reflectance]
    scan = np.fromfile(file_path, dtype=np.float32)
    scan = scan.reshape((-1, 4))
    
    # X, Y, Z coordinates for geometry tracking
    points = scan[:, 0:3]
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd

def preprocess_pcd(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30)
    )
    return pcd_down

def load_calib_tr(file_path):
    """Loads the Velodyne-to-Camera transformation matrix from calib.txt."""
    Tr = np.eye(4)
    with open(file_path, 'r') as f:
        for line in f:
            if line.startswith('Tr:'):
                values = np.array([float(x) for x in line.strip().split()[1:]])
                Tr[:3, :4] = values.reshape(3, 4)
                break
    return Tr

def main():
    # Path to sequence folder
    sequence_dir = "dataset/sequences/00/"
    velo_dir = os.path.join(sequence_dir, "velodyne")
    calib_file = os.path.join(sequence_dir, "calib.txt")
    output_file = "estimated_lidar_trajectory_00.txt"
    
    # Voxel size in meters (e.g., 0.5 means group points into 50cm cubes)
    voxel_size = 0.2 
    
    # All .bin files sorted in alphabetical (chronological) order
    scan_files = sorted(glob.glob(os.path.join(velo_dir, "*.bin")))
    if not scan_files:
        print(f"Error: No .bin files found in {velo_dir}")
        return

    # Load calibration to convert LiDAR coordinates to Camera coordinates
    Tr_velo_to_cam = load_calib_tr(calib_file)
    Tr_cam_to_velo = np.linalg.inv(Tr_velo_to_cam)

    # Initialize tracking variables
    # The starting pose is the identity matrix (0,0,0 position)
    current_pose = np.eye(4) 
    trajectory = [current_pose]
    
    initial_guess = np.eye(4)
    
    # Load and process the very first frame
    print("Loading initial frame...")
    prev_pcd = load_velo_scan(scan_files[0])
    prev_pcd_down = preprocess_pcd(prev_pcd, voxel_size)

    # MAIN ICP LOOP
    print(f"Processing {len(scan_files)} LiDAR scans...")
    
    # Loop through the rest of the frames
    for i in tqdm(range(1, len(scan_files))):
        # 1. Load the new scan
        curr_pcd = load_velo_scan(scan_files[i])
        curr_pcd_down = preprocess_pcd(curr_pcd, voxel_size)

        # 2. Perform Point-to-Plane ICP
        # We align the current frame (source) to the previous frame (target)
        # We use an identity matrix as our initial guess since the car moves slightly between 10Hz frames
        icp_result = o3d.pipelines.registration.registration_icp(
            curr_pcd_down, prev_pcd_down, 
            max_correspondence_distance=0.5,
            init=initial_guess,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100)
        )

        # The transformation matrix from ICP represents the movement in the LiDAR frame
        T_relative_velo = icp_result.transformation

        # Cars have inertia. The motion between Frame 1->2 is used as a
        # prediction for the motion between Frame 2->3.
        initial_guess = T_relative_velo

        # 3. Update the global trajectory in the Velodyne coordinate frame
        current_pose = current_pose @ T_relative_velo

        # 4. Convert the Velodyne pose to the Camera pose for KITTI evaluation
        pose_cam = Tr_velo_to_cam @ current_pose @ Tr_cam_to_velo
        trajectory.append(pose_cam)

        # 5. The current frame becomes the previous frame for the next loop
        prev_pcd_down = curr_pcd_down

    # Save trajectory
    print("Saving trajectory to text file...")
    with open(output_file, 'w') as f:
        for pose in trajectory:
            # Flatten the top 3x4 block of the 4x4 matrix into a single 12-number string
            pose_flat = pose[:3, :4].flatten()
            line = " ".join([f"{val:.6e}" for val in pose_flat])
            f.write(line + "\n")
            
    print(f"Done! Trajectory saved to {output_file}.")

if __name__ == "__main__":
    main()