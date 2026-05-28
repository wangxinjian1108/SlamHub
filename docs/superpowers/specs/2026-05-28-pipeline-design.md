# SlamHub Pipeline Design: Single-LiDAR SLAM → Cross-LiDAR Calibration

Date: 2026-05-28

## Overview

Pipeline for using a single primary LiDAR to run SLAM, then leveraging the generated map to calibrate extrinsics of all other LiDARs on the vehicle.

Target vehicle: TEEMO AW7 with 4 LiDARs (2x Hesai AT128P remote, 2x RoboSense RS-E1R flash) + IMU.

## Workflow

```
Raw Data → [01] Convert → [02] SLAM → [03] Export → [04] Register → [05] Calibrate
                                                         ↑
                                                    viz/ scripts
```

1. Convert primary LiDAR PCD + IMU to rosbag
2. Run FAST-LIO2 on the primary LiDAR bag
3. Export SLAM results to standard intermediate formats
4. Register each secondary LiDAR's point clouds against the primary map
5. Solve refined extrinsics from registration results

## Directory Structure

```
scripts/
├── run_all.sh                       # One-command full pipeline
├── 01_convert_to_rosbag.py          # Raw PCD+IMU → rosbag
├── 02_run_slam.py                   # Launch FAST-LIO2, collect outputs
├── 03_export_slam_results.py        # Extract map + trajectory + per-frame clouds
├── 04_register_secondary.py         # Register secondary LiDAR to primary map
├── 05_solve_extrinsic.py            # Solve final extrinsics from registration
├── registration/                    # Pluggable registration methods
│   ├── base.py                      # Abstract interface
│   └── icp.py                       # ICP implementation (default)
├── common/
│   ├── io.py                        # PCD/pose/yaml read-write utilities
│   └── transform.py                 # Rotation matrix, euler, quaternion conversions
└── viz/
    ├── show_trajectory.py           # 3D trajectory (path + orientation axes)
    ├── show_map.py                  # Point cloud map (color by height/intensity)
    └── show_registration.py         # Before/after registration overlay
```

## Intermediate Output Convention

```
output/<recording_name>/
├── bags/
│   └── <primary_lidar>.bag
├── slam/
│   ├── global_map.pcd               # Full accumulated map
│   ├── trajectory.txt               # TUM format: timestamp tx ty tz qx qy qz qw
│   └── frames/                      # Per-frame clouds in global frame
│       ├── 000000.pcd
│       └── ...
├── registration/
│   ├── <secondary_lidar_1>/
│   │   ├── frame_transforms.txt     # Per-frame registration transforms
│   │   └── summary.yaml             # Stats: mean error, inlier ratio, etc.
│   ├── <secondary_lidar_2>/
│   └── ...
└── calibration/
    └── extrinsics.yaml              # Final calibrated extrinsics (each lidar → primary)
```

## Step Details

### 01_convert_to_rosbag.py (exists)

```bash
python 01_convert_to_rosbag.py <recording_dir> --lidar remote_front_left_pointcloud -o output/<name>/bags/remote_front_left.bag
```

Input: recording directory with `raw_pointclouds/` and `imu.csv`
Output: ROS1 bag with `/points_raw` (PointCloud2) + `/imu/data` (Imu)

### 02_run_slam.py

```bash
python 02_run_slam.py output/<name>/bags/remote_front_left.bag --config config/fastlio_at128p.yaml --output-dir output/<name>/slam/
```

Input: rosbag file + FAST-LIO2 config
Output: SLAM raw outputs (map, trajectory, per-frame data)

Responsibilities:
- Launch FAST-LIO2 node with the provided config
- Play the bag file
- Collect outputs when finished
- Can run inside Docker container or with roslaunch

### 03_export_slam_results.py

```bash
python 03_export_slam_results.py output/<name>/slam/ --format tum
```

Input: SLAM raw output directory
Output: Standardized `global_map.pcd` + `trajectory.txt` + `frames/`

Responsibilities:
- Convert FAST-LIO2 native output to standard formats
- Transform per-frame clouds to global coordinate system using poses
- Save trajectory in TUM format (timestamp tx ty tz qx qy qz qw)

### 04_register_secondary.py

```bash
python 04_register_secondary.py \
    --primary-map output/<name>/slam/global_map.pcd \
    --trajectory output/<name>/slam/trajectory.txt \
    --secondary-dir <recording_dir>/raw_pointclouds/flash_front_pointcloud/ \
    --initial-guess <recording_dir>/application.yaml \
    --method icp \
    --output-dir output/<name>/registration/flash_front/
```

Input: primary map + trajectory + secondary LiDAR raw PCDs + initial extrinsic guess
Output: per-frame transforms + summary statistics

Supports two modes:
- **Global mode**: accumulate secondary clouds into global frame, register against full map
- **Frame mode**: register each secondary frame against local submap around the corresponding pose

The `--method` flag selects the registration backend (icp, ndt, etc.).

### 05_solve_extrinsic.py

```bash
python 05_solve_extrinsic.py \
    --registration-dir output/<name>/registration/ \
    --primary-lidar remote_front_left_pointcloud \
    --output output/<name>/calibration/extrinsics.yaml
```

Input: all registration results
Output: `extrinsics.yaml` with refined transforms for each secondary LiDAR

Responsibilities:
- Aggregate per-frame transforms
- Robust averaging (filter outliers, compute median/mean)
- Output final 4x4 transform or [x,y,z,roll,pitch,yaw] for each LiDAR
- Report confidence metrics (std dev, inlier percentage)

### run_all.sh

```bash
#!/bin/bash
RECORDING_DIR=$1
PRIMARY_LIDAR=${2:-remote_front_left_pointcloud}
OUTPUT_DIR=${3:-output/$(basename $RECORDING_DIR)}

python scripts/01_convert_to_rosbag.py "$RECORDING_DIR" --lidar "$PRIMARY_LIDAR" -o "$OUTPUT_DIR/bags/${PRIMARY_LIDAR}.bag"
python scripts/02_run_slam.py "$OUTPUT_DIR/bags/${PRIMARY_LIDAR}.bag" --output-dir "$OUTPUT_DIR/slam/"
python scripts/03_export_slam_results.py "$OUTPUT_DIR/slam/"

for lidar_dir in "$RECORDING_DIR/raw_pointclouds"/*/; do
    lidar_name=$(basename "$lidar_dir")
    [ "$lidar_name" = "$PRIMARY_LIDAR" ] && continue
    python scripts/04_register_secondary.py \
        --primary-map "$OUTPUT_DIR/slam/global_map.pcd" \
        --trajectory "$OUTPUT_DIR/slam/trajectory.txt" \
        --secondary-dir "$lidar_dir" \
        --initial-guess "$RECORDING_DIR/application.yaml" \
        --method icp \
        --output-dir "$OUTPUT_DIR/registration/$lidar_name/"
done

python scripts/05_solve_extrinsic.py --registration-dir "$OUTPUT_DIR/registration/" --primary-lidar "$PRIMARY_LIDAR" --output "$OUTPUT_DIR/calibration/extrinsics.yaml"
```

## Registration Interface

```python
from dataclasses import dataclass
import numpy as np

@dataclass
class RegistrationResult:
    transformation: np.ndarray   # 4x4 homogeneous transform
    fitness: float               # Overlap ratio [0, 1]
    inlier_rmse: float           # RMS error of inlier correspondences
    num_inliers: int

class RegistrationBase:
    def __init__(self, **kwargs):
        pass

    def register(
        self,
        source: np.ndarray,       # (N, 3) source point cloud
        target: np.ndarray,       # (M, 3) target point cloud
        initial_guess: np.ndarray = None,  # 4x4 initial transform
    ) -> RegistrationResult:
        raise NotImplementedError
```

New methods are added by subclassing `RegistrationBase` and placing the file in `registration/`. The `04_register_secondary.py` script discovers methods by name.

## Visualization Scripts

All viz scripts use Open3D. Each supports:
- Interactive mode (default): opens a 3D viewer window
- Headless mode (`--save <path.png>`): offscreen render to file, no window

### show_trajectory.py

```bash
python viz/show_trajectory.py output/<name>/slam/trajectory.txt [--save traj.png]
```

Renders 3D path as a colored line (color by time), with coordinate axes at sampled poses.

### show_map.py

```bash
python viz/show_map.py output/<name>/slam/global_map.pcd [--color-by height|intensity] [--save map.png]
```

Renders point cloud map with configurable coloring scheme.

### show_registration.py

```bash
python viz/show_registration.py \
    output/<name>/slam/global_map.pcd \
    --secondary output/<name>/registration/flash_front/ \
    --frame 100 \
    [--save reg.png]
```

Overlays primary map (red) with secondary cloud (blue) at a specific frame, showing alignment quality before and after registration.

## Dependencies

- Python 3.8+
- numpy, scipy, open3d, pyyaml
- ROS1 Noetic (for steps 01, 02 only — inside Docker container)
- Steps 03-05 and viz/ are ROS-independent (pure Python + Open3D)

## Design Decisions

1. **ROS-dependent vs ROS-free**: Only steps 01 and 02 require ROS. Everything downstream works with standard PCD + text files, making it easy to run on any machine.
2. **Pluggable registration**: Abstract base class allows swapping methods without touching pipeline logic. Start with ICP, add NDT/feature-based later.
3. **TUM trajectory format**: Widely supported, easy to parse, compatible with evaluation tools (evo, rpg_trajectory_evaluation).
4. **Per-frame + global**: Keeping both per-frame clouds and global map enables both local and global registration strategies.
5. **Headless viz**: Essential for running in Docker/remote servers where no display is available.
