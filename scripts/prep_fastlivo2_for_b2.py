#!/usr/bin/env python3
"""Prep FAST-LIVO2 output for the SlamHub B2 cross-LiDAR pipeline.

FAST-LIVO2 (with our config: `evo.pose_output_en: true`, `seq_name: TEEMO_AT128P`)
writes a TUM-format trajectory at Log/result/TEEMO_AT128P.txt that the
container runner copies to /output/trajectory_lidar.txt:

    time x y z qx qy qz qw    (T_world_lidar)

To plug into 04_register_secondary.py + extract_extrinsic_from_registration.py
(which expect T_world_imu), compose:

    T_world_baselink = T_world_lidar  @ T_lidar_baselink
    T_world_imu      = T_world_baselink @ T_baselink_imu

Same convention as run_kiss_icp.py / prep_liosam_for_b2.py.

Outputs in --livo2-dir:
    trajectory.txt              T_world_imu (B2 input)
    trajectory_baselink.txt     T_world_baselink
    scans_voxel0.3.pcd          voxel-downsampled dense map (B2 ICP target)
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).parent))
from common.transform import (
    euler_to_matrix, quaternion_to_matrix, matrix_to_quaternion,
    make_homogeneous, invert_transform,
)


LIDAR_FRAME = {
    "remote_front_left_pointcloud":  "FRAME_LIDAR_REMOTE_FRONT_LEFT",
    "remote_front_right_pointcloud": "FRAME_LIDAR_REMOTE_FRONT_RIGHT",
    "flash_front_pointcloud":        "FRAME_LIDAR_FLASH_FRONT",
    "flash_rear_pointcloud":         "FRAME_LIDAR_FLASH_REAR",
}


def load_extrinsic(yaml_path, frame_name):
    cfg = yaml.safe_load(open(yaml_path))
    for cal in cfg["vehicle"]["calibration"]["sensor_calibration"]:
        if cal["source"] == frame_name:
            t = cal["transformation"]
            R = euler_to_matrix(t[3], t[4], t[5])
            return make_homogeneous(R, np.array(t[:3]))
    return None


def trajectory_to_imu(traj_lidar, T_lidar_baselink, T_baselink_imu):
    """Compose T_world_lidar → T_world_imu, return (N, 8) TUM rows."""
    out = np.zeros_like(traj_lidar)
    for i, row in enumerate(traj_lidar):
        ts, tx, ty, tz, qx, qy, qz, qw = row
        R = quaternion_to_matrix(qx, qy, qz, qw)
        T_wl = make_homogeneous(R, np.array([tx, ty, tz]))
        T_wi = T_wl @ T_lidar_baselink @ T_baselink_imu
        qx2, qy2, qz2, qw2 = matrix_to_quaternion(T_wi[:3, :3])
        out[i] = [ts, T_wi[0, 3], T_wi[1, 3], T_wi[2, 3], qx2, qy2, qz2, qw2]
    return out


def trajectory_to_baselink(traj_lidar, T_lidar_baselink):
    """Compose T_world_lidar → T_world_baselink, return (N, 8) TUM rows."""
    out = np.zeros_like(traj_lidar)
    for i, row in enumerate(traj_lidar):
        ts, tx, ty, tz, qx, qy, qz, qw = row
        R = quaternion_to_matrix(qx, qy, qz, qw)
        T_wl = make_homogeneous(R, np.array([tx, ty, tz]))
        T_wb = T_wl @ T_lidar_baselink
        qx2, qy2, qz2, qw2 = matrix_to_quaternion(T_wb[:3, :3])
        out[i] = [ts, T_wb[0, 3], T_wb[1, 3], T_wb[2, 3], qx2, qy2, qz2, qw2]
    return out


def write_tum(path, data, comment):
    with open(path, "w") as f:
        f.write(f"# {comment}\n")
        for row in data:
            f.write(" ".join(f"{v:.9f}" for v in row) + "\n")


def voxel_downsample(in_pcd, out_pcd, voxel):
    import open3d as o3d
    pcd = o3d.io.read_point_cloud(str(in_pcd))
    if len(pcd.points) == 0:
        print(f"  WARN: {in_pcd} has 0 points, skipping voxel downsample")
        return False
    ds = pcd.voxel_down_sample(voxel)
    print(f"  voxel {voxel} m: {len(pcd.points):,} → {len(ds.points):,} pts")
    o3d.io.write_point_cloud(str(out_pcd), ds)
    return True


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--livo2-dir", type=Path, required=True,
                   help="Output dir of run_fastlivo2_in_container.sh "
                        "(contains trajectory_lidar.txt + scans*.pcd)")
    p.add_argument("--recording", type=Path, required=True,
                   help="Recording dir with application.yaml")
    p.add_argument("--primary-lidar", default="remote_front_left_pointcloud")
    p.add_argument("--voxel", type=float, default=0.3)
    args = p.parse_args()

    if not args.livo2_dir.exists():
        sys.exit(f"livo2 dir {args.livo2_dir} not found")

    # 1. Load T_world_lidar trajectory
    traj_lidar_path = args.livo2_dir / "trajectory_lidar.txt"
    if not traj_lidar_path.exists():
        sys.exit(f"{traj_lidar_path} missing — did FAST-LIVO2 run produce a trajectory?")
    traj_lidar = np.loadtxt(traj_lidar_path, comments="#")
    if traj_lidar.ndim == 1:
        traj_lidar = traj_lidar.reshape(1, -1)
    print(f"  loaded {len(traj_lidar)} poses from trajectory_lidar.txt")

    # 2. Compose to baselink and IMU using vehicle calibration
    yaml_path = args.recording / "application.yaml"
    frame = LIDAR_FRAME[args.primary_lidar]
    T_baselink_lidar = load_extrinsic(yaml_path, frame)
    if T_baselink_lidar is None:
        sys.exit(f"  no calibration for {frame}")
    T_lidar_baselink = invert_transform(T_baselink_lidar)
    T_baselink_imu = load_extrinsic(yaml_path, "FRAME_GNSS_IMU")
    if T_baselink_imu is None:
        print("  warning: no FRAME_GNSS_IMU; assuming identity")
        T_baselink_imu = np.eye(4)

    traj_baselink = trajectory_to_baselink(traj_lidar, T_lidar_baselink)
    write_tum(args.livo2_dir / "trajectory_baselink.txt", traj_baselink,
              "FAST-LIVO2 T_world_baselink")
    print(f"  trajectory_baselink.txt: {len(traj_baselink)} poses")

    traj_imu = trajectory_to_imu(traj_lidar, T_lidar_baselink, T_baselink_imu)
    write_tum(args.livo2_dir / "trajectory.txt", traj_imu,
              "FAST-LIVO2 T_world_imu (synthetic, for B2 pipeline compat)")
    print(f"  trajectory.txt: {len(traj_imu)} poses")

    # 3. Voxel-downsample dense PCD map. FAST-LIVO2 PCD output is
    # /catkin_ws/src/FAST-LIVO2/PCD/scans.pcd (or similar) — the runner
    # copies it to /output. Find the largest .pcd in livo2-dir.
    out_voxel = args.livo2_dir / f"scans_voxel{args.voxel}.pcd"
    if out_voxel.exists():
        print(f"  {out_voxel} already exists, skipping voxel")
    else:
        candidates = sorted(args.livo2_dir.glob("*.pcd"),
                            key=lambda p: p.stat().st_size, reverse=True)
        candidates = [p for p in candidates if "voxel" not in p.name]
        if not candidates:
            print(f"  no scans*.pcd to downsample in {args.livo2_dir}")
        else:
            src = candidates[0]
            print(f"  voxel-downsampling {src.name} ({src.stat().st_size/1e6:.1f} MB)")
            voxel_downsample(src, out_voxel, args.voxel)
            print(f"  wrote {out_voxel}")


if __name__ == "__main__":
    main()
