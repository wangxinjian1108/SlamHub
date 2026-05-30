#!/usr/bin/env python3
"""Side-by-side before/after registration overlay for each secondary LiDAR.

For each secondary LiDAR:
- BEFORE: transform secondary cloud(s) with FACTORY extrinsic +
  primary IMU pose: T_world_sec = T_world_imu @ T_imu_baselink @ T_baselink_sec_factory
- AFTER : use the per-frame T_world_secondary directly from
  registration/<lidar>/frame_transforms.txt (ICP-refined)

Overlay onto primary map (downsampled). Produces a 2-column PNG.

Usage:
    python3 scripts/viz_registration_compare.py \
        --output-dir output/ghcr_run_v3 \
        --recording-dir <path>/raw_pointclouds_parent \
        --num-frames 30
"""
import argparse
import sys
from pathlib import Path
import numpy as np
import yaml
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).parent))
from common.io import read_pcd, read_trajectory_tum
from common.transform import (
    euler_to_matrix, quaternion_to_matrix, make_homogeneous, invert_transform,
)

LIDAR_FRAME = {
    "remote_front_right_pointcloud": "FRAME_LIDAR_REMOTE_FRONT_RIGHT",
    "flash_front_pointcloud": "FRAME_LIDAR_FLASH_FRONT",
    "flash_rear_pointcloud": "FRAME_LIDAR_FLASH_REAR",
}


def load_factory_T_baselink_sec(yaml_path, frame):
    cfg = yaml.safe_load(open(yaml_path))
    for cal in cfg["vehicle"]["calibration"]["sensor_calibration"]:
        if cal["source"] == frame:
            t = cal["transformation"]
            R = euler_to_matrix(t[3], t[4], t[5])
            return make_homogeneous(R, np.array(t[:3]))
    return None


def pose_row_to_T(pose):
    _, tx, ty, tz, qx, qy, qz, qw = pose
    return make_homogeneous(quaternion_to_matrix(qx, qy, qz, qw), np.array([tx, ty, tz]))


def load_frame_transforms(path):
    """Return dict ts_ns -> 4x4."""
    out = {}
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
            out[ts] = T
    return out


def transform_xyz(T, xyz):
    h = np.hstack([xyz, np.ones((len(xyz), 1))])
    return (T @ h.T).T[:, :3]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--output-dir", type=Path, required=True,
                   help="ghcr_run_v3 dir with scans_voxel0.3.pcd, trajectory.txt, registration/")
    p.add_argument("--recording-dir", type=Path, required=True,
                   help="raw recording dir containing raw_pointclouds/ and application.yaml")
    p.add_argument("--num-frames", type=int, default=30)
    p.add_argument("--max-pts-per-frame", type=int, default=4000)
    p.add_argument("--map-voxel-pts", type=int, default=200000)
    p.add_argument("--zoom-radius", type=float, default=120.0,
                   help="auto-crop view to ±radius around trajectory centroid")
    args = p.parse_args()

    out_dir = args.output_dir
    rec_dir = args.recording_dir

    print("Loading map...")
    primary_map = read_pcd(out_dir / "scans_voxel0.3.pcd")
    if len(primary_map) > args.map_voxel_pts:
        idx = np.random.choice(len(primary_map), args.map_voxel_pts, replace=False)
        primary_map = primary_map[idx]
    print(f"  {len(primary_map):,} map points (subsampled)")

    poses = read_trajectory_tum(out_dir / "trajectory.txt")
    pose_ts_ns = (poses[:, 0] * 1e9).astype(np.int64)
    pose_Ts = [pose_row_to_T(r) for r in poses]
    centroid = poses[:, 1:4].mean(axis=0)
    xlim = (centroid[0] - args.zoom_radius, centroid[0] + args.zoom_radius)
    ylim = (centroid[1] - args.zoom_radius, centroid[1] + args.zoom_radius)

    T_baselink_imu = load_factory_T_baselink_sec(
        rec_dir / "application.yaml", "FRAME_GNSS_IMU")
    T_imu_baselink = invert_transform(T_baselink_imu)
    print(f"baselink<-imu translation: {T_baselink_imu[:3,3]}")

    for lidar, frame in LIDAR_FRAME.items():
        reg_dir = out_dir / "registration" / lidar
        ft_path = reg_dir / "frame_transforms.txt"
        if not ft_path.exists():
            print(f"skip {lidar}: no frame_transforms.txt")
            continue
        print(f"\n=== {lidar} ===")

        T_baselink_sec_factory = load_factory_T_baselink_sec(
            rec_dir / "application.yaml", frame)
        T_imu_sec_factory = T_imu_baselink @ T_baselink_sec_factory

        refined = load_frame_transforms(ft_path)
        sec_pcd_dir = rec_dir / "raw_pointclouds" / lidar
        all_pcds = sorted(sec_pcd_dir.glob("*.pcd"))
        # Sample evenly
        idx = np.linspace(0, len(all_pcds) - 1, args.num_frames).astype(int)
        samples = [all_pcds[i] for i in idx]
        print(f"  sampling {len(samples)} frames from {len(all_pcds)}")

        before_pts, after_pts = [], []
        for pcd_file in samples:
            ts = int(pcd_file.stem)
            if ts not in refined:
                # match by closest
                closest_ts = min(refined.keys(), key=lambda x: abs(x - ts))
                T_after = refined[closest_ts]
            else:
                T_after = refined[ts]
            # nearest primary pose
            pi = int(np.argmin(np.abs(pose_ts_ns - ts)))
            T_world_imu = pose_Ts[pi]
            T_before = T_world_imu @ T_imu_sec_factory

            xyz = read_pcd(pcd_file)
            xyz = xyz[~np.isnan(xyz[:, 0])][:, :3]
            if len(xyz) > args.max_pts_per_frame:
                sel = np.random.choice(len(xyz), args.max_pts_per_frame, replace=False)
                xyz = xyz[sel]

            before_pts.append(transform_xyz(T_before, xyz))
            after_pts.append(transform_xyz(T_after, xyz))

        before_pts = np.vstack(before_pts)
        after_pts = np.vstack(after_pts)

        # Mean shift between before and after, for diagnostic
        delta = after_pts.mean(0) - before_pts.mean(0)
        print(f"  centroid shift after-before: dx={delta[0]:.3f} dy={delta[1]:.3f} dz={delta[2]:.3f}")

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10), sharex=True, sharey=True)
        for ax, sec_pts, title in [
            (ax1, before_pts, "BEFORE (factory extrinsic)"),
            (ax2, after_pts, "AFTER (ICP-calibrated)"),
        ]:
            ax.scatter(primary_map[:, 0], primary_map[:, 1],
                       c="lightgray", s=0.4, label="primary map", rasterized=True)
            ax.scatter(sec_pts[:, 0], sec_pts[:, 1],
                       c="red", s=0.8, alpha=0.4, label=f"{lidar}", rasterized=True)
            ax.plot(poses[:, 1], poses[:, 2], "b-", lw=1.0, alpha=0.7, label="trajectory")
            ax.set_xlim(*xlim); ax.set_ylim(*ylim)
            ax.set_aspect("equal"); ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
            ax.set_title(title); ax.legend(loc="upper left", fontsize=9)

        fig.suptitle(f"{lidar}: before/after cross-LiDAR ICP calibration ({len(samples)} frames)",
                     fontsize=14)
        out_png = out_dir / f"compare_{lidar}.png"
        fig.savefig(out_png, dpi=130, bbox_inches="tight")
        plt.close(fig)
        print(f"  saved {out_png}")


if __name__ == "__main__":
    main()
