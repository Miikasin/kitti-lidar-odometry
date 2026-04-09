# Robust Frame-to-Frame LiDAR Odometry

![KITTI Benchmark](https://img.shields.io/badge/Dataset-KITTI_Odometry-blue)
![Language](https://img.shields.io/badge/Language-Python_|_C++-success)
![Library](https://img.shields.io/badge/Library-Open3D-orange)

A highly optimized, pure frame-to-frame LiDAR odometry pipeline utilizing Point-to-Plane Iterative Closest Point (ICP) and a predictive Constant Velocity motion model. Evaluated on the KITTI Odometry Benchmark sequences 00-10.

## Key Results
This system was specifically optimized for high-speed urban driving environments. By mitigating the classic "high-speed ICP divergence" problem, the pure frame-to-frame architecture achieved highly competitive results without the use of a SLAM backend, loop closure, or IMU sensor fusion.

* **Translation Error:** `1.30 %`
* **Rotation Error:** `0.0057 deg/m`

<img width="500" height="250" alt="avg_rl" src="https://github.com/user-attachments/assets/4d216516-3582-4c3d-bdf2-8b25676d8323" />
<img width="500" height="250" alt="avg_tl" src="https://github.com/user-attachments/assets/5c067ac0-bf90-4ba1-910b-6aea4319f798" />

## Methodology & Pipeline
The core tracker is written in Python using `Open3D` and follows a strict sequential pipeline:

1. **Preprocessing & Downsampling:** Raw KITTI `.bin` point clouds (>100k points) are isolated for $X, Y, Z$ spatial geometry and downsampled using a Voxel Grid Filter (leaf size: `0.2m`) to ensure real-time computational efficiency.
2. **Normal Estimation:** Surface normals are computed via KD-Tree search (radius: `0.4m`, max neighbors: `30`) as a prerequisite for Point-to-Plane alignment.
3. **Constant Velocity Prediction:** To prevent ICP divergence when the vehicle accelerates above 30 km/h, the system assumes inertia remains constant between 10Hz frames. The relative transformation matrix $\mathbf{T}_{relative}$ from frame $t-1$ is injected as the initial guess for frame $t$.
4. **Point-to-Plane ICP:** Ego-motion is calculated using a tight `0.5m` maximum correspondence distance. This narrow search radius acts as a spatial scalpel, locking onto high-quality structural geometry (walls, roads) while naturally ignoring outliers and moving vehicles.
5. **Global Accumulation:** Relative transformations are accumulated into a global trajectory and projected into the left camera coordinate frame using static calibration matrices ($\mathbf{T}_{velo \to cam}$) for standardized KITTI evaluation.

## Installation & Requirements
**Python Dependencies:**
```bash
pip install numpy open3d tqdm
```

## How to Run
1. **Download the Data:** Download the official KITTI Odometry dataset (specifically `data_odometry_velodyne.zip` and `data_odometry_calib.zip`) and place Sequence `00` inside a `dataset/sequences/00/` directory.

2. **Run the Tracker:**
```bash
python Lidar_SLAM.py
```

## 3. Evaluate the Results:
Ensure the official KITTI ground truth poses are located in `devkit/cpp/data/odometry/poses/`.
```bash
cd devkit/cpp
./eval_odometry <name_of_your_results_folder>
```

## Project Structure
```text
├── Lidar_SLAM.py               # Main Python odometry tracker
├── dataset/                    # (Not included) KITTI sequence data
├── devkit/                     # C++ Evaluation scripts and Ground Truth
│   ├── cpp/
│   │   ├── evaluate_odometry.cpp 
│   │   ├── matrix.cpp 
│   │   └── data/odometry/poses/ # Ground truth .txt files go here
├── results/                    # Output directory for generated trajectories and plots
└── README.md
```
