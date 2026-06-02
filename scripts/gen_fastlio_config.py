#!/usr/bin/env python3
"""Generate a FAST-LIO2 YAML config for a given recording.

FAST-LIO's extrinsic_T / extrinsic_R encode T_imu_lidar (LiDAR pose in IMU
frame). Per-vehicle values vary across recordings, so we read from
application.yaml and generate a tailored config.

Usage:
  python gen_fastlio_config.py <recording> <out-yaml> [--lidar remote_front_left_pointcloud]
"""
import argparse
import sys
from pathlib import Path
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).parent))
from common.transform import euler_to_matrix, make_homogeneous, invert_transform

LIDAR_FRAME = {
    "remote_front_left_pointcloud":  "FRAME_LIDAR_REMOTE_FRONT_LEFT",
    "remote_front_right_pointcloud": "FRAME_LIDAR_REMOTE_FRONT_RIGHT",
    "flash_front_pointcloud":        "FRAME_LIDAR_FLASH_FRONT",
    "flash_rear_pointcloud":         "FRAME_LIDAR_FLASH_REAR",
}


def load_calibration(application_yaml, frame):
    cfg = yaml.safe_load(open(application_yaml))
    for cal in cfg["vehicle"]["calibration"]["sensor_calibration"]:
        if cal.get("source") == frame:
            t = cal["transformation"]
            R = euler_to_matrix(t[3], t[4], t[5])
            return make_homogeneous(R, np.array(t[:3]))
    return None


def main():
    p = argparse.ArgumentParser()
    p.add_argument("recording", type=Path)
    p.add_argument("out", type=Path)
    p.add_argument("--lidar", default="remote_front_left_pointcloud")
    args = p.parse_args()

    yaml_path = args.recording / "application.yaml"
    frame = LIDAR_FRAME[args.lidar]
    T_baselink_lidar = load_calibration(yaml_path, frame)
    T_baselink_imu   = load_calibration(yaml_path, "FRAME_GNSS_IMU")
    if T_baselink_lidar is None or T_baselink_imu is None:
        sys.exit(f"missing calibration for {frame} or FRAME_GNSS_IMU")

    T_imu_lidar = invert_transform(T_baselink_imu) @ T_baselink_lidar
    R = T_imu_lidar[:3, :3]
    t = T_imu_lidar[:3, 3]

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with open(args.out, "w") as f:
        f.write(f"""\
common:
    lid_topic:  "/velodyne_points"
    imu_topic:  "/imu/data"
    time_sync_en: false
    time_offset_lidar_to_imu: 0.0

preprocess:
    lidar_type: 2
    scan_line: 128
    scan_rate: 10
    timestamp_unit: 0
    blind: 1.0

mapping:
    acc_cov: 0.1
    gyr_cov: 0.1
    b_acc_cov: 0.0001
    b_gyr_cov: 0.0001
    fov_degree:    180
    det_range:     150.0
    extrinsic_est_en:  false
    # Auto-generated from {yaml_path.name} for primary LiDAR={args.lidar}.
    # extrinsic_T / extrinsic_R = T_imu_lidar (LiDAR pose in IMU frame).
    extrinsic_T: [ {t[0]:.6f}, {t[1]:.6f}, {t[2]:.6f} ]
    extrinsic_R: [ {R[0,0]: .6f}, {R[0,1]: .6f}, {R[0,2]: .6f},
                   {R[1,0]: .6f}, {R[1,1]: .6f}, {R[1,2]: .6f},
                   {R[2,0]: .6f}, {R[2,1]: .6f}, {R[2,2]: .6f} ]

publish:
    path_en:  true
    scan_publish_en:  true
    dense_publish_en: true
    scan_bodyframe_pub_en: true

pcd_save:
    pcd_save_en: true
    interval: -1
""")
    print(f"Wrote {args.out}")
    print(f"  T_imu_lidar = [{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}]")


if __name__ == "__main__":
    main()
