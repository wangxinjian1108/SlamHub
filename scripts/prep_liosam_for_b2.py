#!/usr/bin/env python3
"""Prep LIO-SAM output for the SlamHub B2 cross-LiDAR pipeline.

LIO-SAM (with our config: lidarFrame=baselinkFrame=base_link, PointCloud2
delivered in lidar_link frame via identity static TF) publishes poses that
are effectively T_world_lidar — same convention as KISS-ICP. To plug into
the existing 04_register_secondary.py + extract_extrinsic_from_registration.py
pipeline, produce:

  trajectory.txt       T_world_imu, TUM format       (B2 expects this)
  scans_voxel0.3.pcd   primary map, voxel 0.3 m       (ICP target)

Composition (mirrors run_kiss_icp.py):
  T_world_baselink = T_world_lidar  @ T_lidar_baselink
  T_world_imu      = T_world_baselink @ T_baselink_imu

Outputs are written next to the existing LIO-SAM run files
(GlobalMap.pcd, transformations.pcd) in --output-dir.
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
from extract_liosam_trajectory import parse_pcd_xyzirpyt, rpy_to_quat


LIDAR_FRAME = {
    "remote_front_left_pointcloud": "FRAME_LIDAR_REMOTE_FRONT_LEFT",
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


def slerp(q0, q1, t):
    """Spherical linear interpolation between two unit quaternions (xyzw)."""
    q0 = np.asarray(q0); q1 = np.asarray(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1; dot = -dot
    if dot > 0.9995:
        out = q0 + t * (q1 - q0)
        return out / np.linalg.norm(out)
    theta = np.arccos(np.clip(dot, -1.0, 1.0))
    s0 = np.sin((1 - t) * theta) / np.sin(theta)
    s1 = np.sin(t * theta) / np.sin(theta)
    return s0 * q0 + s1 * q1


def interpolate_to_timestamps(traj, target_ts):
    """Resample a sparse keyframe trajectory at target timestamps (seconds).
    SLERP for rotation, linear for translation. Outside the keyframe range:
    extrapolate by clamping to the nearest endpoint."""
    src_ts = traj[:, 0]
    out = np.zeros((len(target_ts), 8))
    out[:, 0] = target_ts
    n = len(traj)
    j = 0  # index into src
    for k, t in enumerate(target_ts):
        # advance j so src_ts[j] <= t < src_ts[j+1]
        while j + 1 < n and src_ts[j + 1] < t:
            j += 1
        if t <= src_ts[0]:
            out[k, 1:] = traj[0, 1:]
        elif t >= src_ts[-1]:
            out[k, 1:] = traj[-1, 1:]
        else:
            t0, t1 = src_ts[j], src_ts[j + 1]
            alpha = (t - t0) / (t1 - t0)
            out[k, 1:4] = (1 - alpha) * traj[j, 1:4] + alpha * traj[j + 1, 1:4]
            q = slerp(traj[j, 4:8], traj[j + 1, 4:8], alpha)
            out[k, 4:8] = q
    return out


def write_tum(path, data, comment):
    with open(path, "w") as f:
        f.write(f"# {comment}\n")
        for row in data:
            f.write(" ".join(f"{v:.9f}" for v in row) + "\n")


def voxel_downsample(in_pcd, out_pcd, voxel):
    import open3d as o3d
    pcd = o3d.io.read_point_cloud(str(in_pcd))
    ds = pcd.voxel_down_sample(voxel)
    print(f"  voxel {voxel} m: {len(pcd.points):,} → {len(ds.points):,} pts")
    o3d.io.write_point_cloud(str(out_pcd), ds)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--liosam-dir", type=Path, default=Path("output/liosam_run"))
    p.add_argument("--recording", type=Path,
                   default=Path("/root/node_data/fixtures/lio/ZL11626_40482_zelos_sample_2025-07-02_14-29-00_000000000_8993544"))
    p.add_argument("--primary-lidar", default="remote_front_left_pointcloud")
    p.add_argument("--voxel", type=float, default=0.3)
    args = p.parse_args()

    # 1. Build T_world_lidar from transformations.pcd
    tx = parse_pcd_xyzirpyt(args.liosam_dir / "transformations.pcd")
    qx, qy, qz, qw = rpy_to_quat(tx["roll"], tx["pitch"], tx["yaw"])
    traj_lidar_kf = np.column_stack([
        tx["time"], tx["x"], tx["y"], tx["z"], qx, qy, qz, qw,
    ])
    write_tum(args.liosam_dir / "trajectory_lidar_keyframes.txt", traj_lidar_kf,
              "LIO-SAM T_world_lidar (keyframes only)")
    print(f"  trajectory_lidar_keyframes.txt: {len(traj_lidar_kf)} keyframes")

    # 1b. Interpolate to PCD timestamps (10 Hz, 600 frames) — B2 expects per-frame
    # poses to look up T_world_base for each secondary frame; LIO-SAM's 286
    # sparse keyframes would mis-attribute pose by up to ±100 ms otherwise.
    pcd_dir = args.recording / "raw_pointclouds" / args.primary_lidar
    pcd_ts = np.array(sorted(int(f.stem) for f in pcd_dir.glob("*.pcd"))) / 1e9
    traj_lidar = interpolate_to_timestamps(traj_lidar_kf, pcd_ts)
    write_tum(args.liosam_dir / "trajectory_lidar.txt", traj_lidar,
              "LIO-SAM T_world_lidar (interpolated to PCD timestamps)")
    print(f"  trajectory_lidar.txt: {len(traj_lidar)} dense poses (interpolated)")

    # 2. Compose to baselink → IMU using vehicle calibration
    yaml_path = args.recording / "application.yaml"
    frame = LIDAR_FRAME[args.primary_lidar]
    T_baselink_lidar = load_extrinsic(yaml_path, frame)
    if T_baselink_lidar is None:
        sys.exit(f"  no calibration for {frame}")
    T_lidar_baselink = invert_transform(T_baselink_lidar)
    T_baselink_imu = load_extrinsic(yaml_path, "FRAME_GNSS_IMU")
    if T_baselink_imu is None:
        print("  warning: no FRAME_GNSS_IMU; assuming identity (IMU==baselink)")
        T_baselink_imu = np.eye(4)
    traj_imu = trajectory_to_imu(traj_lidar, T_lidar_baselink, T_baselink_imu)
    write_tum(args.liosam_dir / "trajectory_imu.txt", traj_imu,
              "LIO-SAM T_world_imu (synthetic, for B2 pipeline compat)")
    write_tum(args.liosam_dir / "trajectory.txt", traj_imu,
              "LIO-SAM T_world_imu (synthetic, for B2 pipeline compat)")
    print(f"  trajectory_imu.txt + trajectory.txt: {len(traj_imu)} keyframes")

    # 3. Voxel-downsample GlobalMap.pcd → scans_voxel0.3.pcd
    out_voxel = args.liosam_dir / f"scans_voxel{args.voxel}.pcd"
    voxel_downsample(args.liosam_dir / "GlobalMap.pcd", out_voxel, args.voxel)
    print(f"  wrote {out_voxel}")


if __name__ == "__main__":
    main()
