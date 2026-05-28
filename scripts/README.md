# scripts

SlamHub pipeline scripts for single-LiDAR SLAM → cross-LiDAR calibration.

## Pipeline Steps

| Step | Script | Description | Requires ROS |
|------|--------|-------------|:---:|
| 01 | `01_convert_to_rosbag.py` | Raw PCD+IMU → rosbag | Yes |
| 02 | `02_run_slam.py` | Run FAST-LIO2 | Yes |
| 03 | `03_export_slam_results.py` | Export map + trajectory + frames | No |
| 04 | `04_register_secondary.py` | Register secondary LiDARs | No |
| 05 | `05_solve_extrinsic.py` | Solve calibrated extrinsics | No |

## Quick Start

```bash
# Full pipeline (inside Docker with ROS)
./run_all.sh /path/to/recording remote_front_left_pointcloud

# Or run steps individually
python 01_convert_to_rosbag.py /path/to/recording --list-lidars
python 01_convert_to_rosbag.py /path/to/recording --lidar remote_front_left_pointcloud -o output/bags/primary.bag
```

## Visualization

```bash
python viz/show_trajectory.py output/slam/trajectory.txt
python viz/show_map.py output/slam/global_map.pcd --color-by height
python viz/show_registration.py output/slam/global_map.pcd --secondary output/registration/flash_front/ --frame 50
```

All viz scripts support `--save <path.png>` for headless rendering.

## Dependencies

```bash
pip install -r requirements.txt
```

Steps 01-02 require ROS1 Noetic (run inside FAST_LIO Docker container).
Steps 03-05 and viz/ are ROS-independent.
