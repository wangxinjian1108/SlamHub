#!/usr/bin/env python3
"""Generate a LIO-SAM YAML config for a given recording.

LIO-SAM requires extrinsicTrans/extrinsicRot pinned to per-vehicle values.
Each sample in node_data/fixtures/lio has its own application.yaml. This
script reads the primary LiDAR + IMU extrinsics from that YAML and emits a
matching `liosam_<lidar>.yaml` next to the base config.

Convention recap (verified for ZL11626 in §17.2):
  - LIO-SAM imuConverter applies `acc_lidar = extRot * acc_imu`
  - So extrinsicRot = R_lidar_imu = (R_imu_lidar)^T
  - R_imu_lidar = R_imu_baselink @ R_baselink_lidar
  - Translation: extrinsicTrans = same as the FAST-LIO config (T_imu_lidar)

Usage:
  python gen_liosam_config.py <recording> <out-yaml> [--lidar remote_front_left_pointcloud]
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

BASE_CONFIG = """\
lio_sam:
  pointCloudTopic: "/velodyne_points"
  imuTopic: "/imu/data"
  odomTopic: "odometry/imu"
  gpsTopic: "odometry/gpsz"

  lidarFrame: "base_link"
  baselinkFrame: "base_link"
  odometryFrame: "odom"
  mapFrame: "map"

  useImuHeadingInitialization: false
  useGpsElevation: false
  gpsCovThreshold: 2.0
  poseCovThreshold: 25.0

  savePCD: true
  savePCDDirectory: "/output/"

  sensor: velodyne
  N_SCAN: 128
  Horizon_SCAN: 1800
  downsampleRate: 1
  lidarMinRange: 1.0
  lidarMaxRange: 200.0

  imuAccNoise: 3.9939570888238808e-03
  imuGyrNoise: 1.5636343949698187e-03
  imuAccBiasN: 6.4356659353532566e-05
  imuGyrBiasN: 3.5640318696367613e-05
  imuGravity: 9.80511
  imuRPYWeight: 0.01

  edgeThreshold: 1.0
  surfThreshold: 0.1
  edgeFeatureMinValidNum: 10
  surfFeatureMinValidNum: 100

  odometrySurfLeafSize: 0.4
  mappingCornerLeafSize: 0.2
  mappingSurfLeafSize: 0.4

  z_tollerance: 1000
  rotation_tollerance: 1000

  numberOfCores: 4
  mappingProcessInterval: 0.15

  surroundingkeyframeAddingDistThreshold: 1.0
  surroundingkeyframeAddingAngleThreshold: 0.2
  surroundingKeyframeDensity: 2.0
  surroundingKeyframeSearchRadius: 50.0

  loopClosureEnableFlag: true
  loopClosureFrequency: 1.0
  surroundingKeyframeSize: 50
  historyKeyframeSearchRadius: 15.0
  historyKeyframeSearchTimeDiff: 30.0
  historyKeyframeSearchNum: 25
  historyKeyframeFitnessScore: 0.3

  globalMapVisualizationSearchRadius: 1000.0
  globalMapVisualizationPoseDensity: 10.0
  globalMapVisualizationLeafSize: 1.0
"""


def load_calibration(application_yaml: Path, frame: str):
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
        sys.exit(f"missing calibration for {frame} or FRAME_GNSS_IMU in {yaml_path}")

    # T_imu_lidar = inv(T_baselink_imu) @ T_baselink_lidar
    T_imu_lidar = invert_transform(T_baselink_imu) @ T_baselink_lidar
    R_imu_lidar = T_imu_lidar[:3, :3]
    R_lidar_imu = R_imu_lidar.T   # what LIO-SAM expects in extrinsicRot
    t_imu_lidar = T_imu_lidar[:3, 3]

    def fmt_row(r):
        return f"[{r[0,0]: .6f}, {r[0,1]: .6f}, {r[0,2]: .6f},\n" \
               f"                 {r[1,0]: .6f}, {r[1,1]: .6f}, {r[1,2]: .6f},\n" \
               f"                 {r[2,0]: .6f}, {r[2,1]: .6f}, {r[2,2]: .6f}]"

    extrinsic_block = (
        f"\n  # Auto-generated from {yaml_path.name} for primary LiDAR={args.lidar}.\n"
        f"  # extrinsicTrans = T_imu_lidar (translation of LiDAR origin in IMU frame).\n"
        f"  # extrinsicRot = R_lidar_imu = (R_imu_lidar)^T  (LIO-SAM convention).\n"
        f"  extrinsicTrans: [{t_imu_lidar[0]:.6f}, {t_imu_lidar[1]:.6f}, {t_imu_lidar[2]:.6f}]\n"
        f"  extrinsicRot: {fmt_row(R_lidar_imu)}\n"
        f"  extrinsicRPY: {fmt_row(R_lidar_imu)}\n"
    )

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with open(args.out, "w") as f:
        f.write(BASE_CONFIG)
        f.write(extrinsic_block)
    print(f"Wrote {args.out}")
    print(f"  T_imu_lidar = [{t_imu_lidar[0]:.4f}, {t_imu_lidar[1]:.4f}, {t_imu_lidar[2]:.4f}]")


if __name__ == "__main__":
    main()
