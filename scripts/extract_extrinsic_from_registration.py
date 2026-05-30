#!/usr/bin/env python3
"""Compute calibrated extrinsic T_baselink_secondary from 04 registration output.

Frame_transforms.txt rows are T_world_secondary (refined by ICP).
Primary trajectory.txt gives T_world_baselink at each pose timestamp.
For each frame: T_baselink_secondary = T_world_baselink^-1 @ T_world_secondary.
Aggregate per-frame extrinsics with median translation + average quaternion.

Compare to the YAML initial guess and report delta.

Usage:
    python3 scripts/extract_extrinsic_from_registration.py \
        --primary-trajectory output/ghcr_run_v3/trajectory.txt \
        --registration-dir output/ghcr_run_v3/registration \
        --initial-guess <recording>/application.yaml \
        --output output/ghcr_run_v3/calibrated_extrinsics.yaml
"""
import argparse
import sys
from pathlib import Path
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).parent))
from common.transform import (
    euler_to_matrix, matrix_to_euler,
    matrix_to_quaternion, quaternion_to_matrix,
    invert_transform, make_homogeneous,
)
from common.io import read_trajectory_tum

LIDAR_FRAME = {
    "remote_front_left_pointcloud": "FRAME_LIDAR_REMOTE_FRONT_LEFT",
    "remote_front_right_pointcloud": "FRAME_LIDAR_REMOTE_FRONT_RIGHT",
    "flash_front_pointcloud": "FRAME_LIDAR_FLASH_FRONT",
    "flash_rear_pointcloud": "FRAME_LIDAR_FLASH_REAR",
}


def pose_to_T(pose):
    _, tx, ty, tz, qx, qy, qz, qw = pose
    R = quaternion_to_matrix(qx, qy, qz, qw)
    return make_homogeneous(R, np.array([tx, ty, tz]))


def load_frame_transforms(path):
    ts_list, T_list = [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            ts = int(parts[0])
            vals = [float(x) for x in parts[1:13]]
            T = np.eye(4)
            T[:3, :] = np.array(vals).reshape(3, 4)
            ts_list.append(ts)
            T_list.append(T)
    return np.array(ts_list), T_list


def load_yaml_extrinsic_by_frame(yaml_path, frame_name):
    with open(yaml_path) as f:
        cfg = yaml.safe_load(f)
    for cal in cfg["vehicle"]["calibration"]["sensor_calibration"]:
        if cal["source"] == frame_name:
            t = cal["transformation"]
            R = euler_to_matrix(t[3], t[4], t[5])
            return make_homogeneous(R, np.array([t[0], t[1], t[2]]))
    return None


def load_yaml_extrinsic(yaml_path, secondary_name):
    frame = LIDAR_FRAME.get(secondary_name)
    if not frame:
        return None
    return load_yaml_extrinsic_by_frame(yaml_path, frame)


def aggregate(transforms, fitness=None, fitness_threshold=0.5):
    """Median translation, sign-consistent quaternion average."""
    keep = transforms
    if fitness is not None:
        mask = np.array(fitness) > fitness_threshold
        keep = [T for T, m in zip(transforms, mask) if m]
        print(f"  kept {len(keep)}/{len(transforms)} with fitness > {fitness_threshold}")
    if not keep:
        return np.eye(4), 0

    translations = np.array([T[:3, 3] for T in keep])
    median_t = np.median(translations, axis=0)
    std_t = np.std(translations, axis=0)

    qs = []
    for T in keep:
        q = matrix_to_quaternion(T[:3, :3])
        qs.append(np.array([q[0], q[1], q[2], q[3]]))
    qs = np.array(qs)
    for i in range(1, len(qs)):
        if np.dot(qs[i], qs[0]) < 0:
            qs[i] = -qs[i]
    mean_q = qs.mean(axis=0)
    mean_q = mean_q / np.linalg.norm(mean_q)

    R = quaternion_to_matrix(*mean_q)
    T_out = make_homogeneous(R, median_t)
    return T_out, std_t


def load_fitness(summary_dir):
    """Load per-frame fitness if available (from 04 stdout log would be ideal,
    but only summary.yaml has aggregate stats). Return None to skip filtering."""
    return None


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--primary-trajectory", type=Path, required=True)
    p.add_argument("--registration-dir", type=Path, required=True)
    p.add_argument("--initial-guess", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()

    poses = read_trajectory_tum(args.primary_trajectory)
    pose_ts = (poses[:, 0] * 1e9).astype(np.int64)
    pose_Ts = [pose_to_T(p) for p in poses]

    # FAST-LIO trajectory is T_world_IMU, not T_world_baselink. To recover
    # T_baselink_secondary we compose T_baselink_IMU @ T_imu_secondary.
    T_baselink_imu = load_yaml_extrinsic_by_frame(args.initial_guess, "FRAME_GNSS_IMU")
    if T_baselink_imu is None:
        print("Warning: FRAME_GNSS_IMU not found; assuming IMU == baselink.")
        T_baselink_imu = np.eye(4)
    else:
        print(f"IMU offset (baselink frame): t={T_baselink_imu[:3,3]}")

    result = {"calibrated_extrinsics": {}}

    for reg_subdir in sorted(args.registration_dir.iterdir()):
        if not reg_subdir.is_dir():
            continue
        name = reg_subdir.name
        ft_path = reg_subdir / "frame_transforms.txt"
        if not ft_path.exists():
            continue

        print(f"\n=== {name} ===")
        ts_list, T_world_sec_list = load_frame_transforms(ft_path)

        # Per-frame extract T_imu_secondary = T_world_imu^-1 @ T_world_secondary
        # then express in baselink frame: T_baselink_secondary = T_baselink_imu @ T_imu_secondary
        extrinsics = []
        for ts_ns, T_ws in zip(ts_list, T_world_sec_list):
            idx = int(np.argmin(np.abs(pose_ts - ts_ns)))
            T_wi = pose_Ts[idx]
            T_is = invert_transform(T_wi) @ T_ws
            T_bs = T_baselink_imu @ T_is
            extrinsics.append(T_bs)

        T_calib, std_t = aggregate(extrinsics)
        tx, ty, tz = T_calib[:3, 3]
        roll, pitch, yaw = matrix_to_euler(T_calib[:3, :3])

        T_init = load_yaml_extrinsic(args.initial_guess, name)
        info = {
            "translation_xyz_m": [float(tx), float(ty), float(tz)],
            "euler_rpy_rad": [float(roll), float(pitch), float(yaw)],
            "translation_std_m": [float(s) for s in std_t],
            "num_frames": len(extrinsics),
        }
        if T_init is not None:
            dt = T_calib[:3, 3] - T_init[:3, 3]
            ir, ip, iy = matrix_to_euler(T_init[:3, :3])
            info["initial_xyz_rpy"] = [float(T_init[0, 3]), float(T_init[1, 3]),
                                       float(T_init[2, 3]),
                                       float(ir), float(ip), float(iy)]
            info["delta_translation_m"] = [float(dt[0]), float(dt[1]), float(dt[2])]
            info["delta_translation_norm_m"] = float(np.linalg.norm(dt))
            print(f"  initial : t=[{T_init[0,3]:7.4f},{T_init[1,3]:7.4f},{T_init[2,3]:7.4f}] "
                  f"rpy=[{ir:+.4f},{ip:+.4f},{iy:+.4f}]")
        print(f"  calib   : t=[{tx:7.4f},{ty:7.4f},{tz:7.4f}] "
              f"rpy=[{roll:+.4f},{pitch:+.4f},{yaw:+.4f}]")
        print(f"  std_t   : [{std_t[0]:.4f},{std_t[1]:.4f},{std_t[2]:.4f}] m")
        if T_init is not None:
            print(f"  Δt      : [{dt[0]:+.4f},{dt[1]:+.4f},{dt[2]:+.4f}] m "
                  f"(|Δt|={np.linalg.norm(dt):.4f} m)")

        result["calibrated_extrinsics"][name] = info

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with open(args.output, "w") as f:
        yaml.dump(result, f, default_flow_style=None, sort_keys=False)
    print(f"\nWrote {args.output}")


if __name__ == "__main__":
    main()
