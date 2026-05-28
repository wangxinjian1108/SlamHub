#!/usr/bin/env python3
"""
Export FAST-LIO2 raw outputs to standardized formats.

Converts FAST-LIO2 PCD directory outputs into:
  - global_map.pcd (merged map)
  - trajectory.txt (TUM format)
  - frames/ (per-frame clouds in global coordinates)

Usage:
    python 03_export_slam_results.py output/slam/ --output-dir output/slam/
"""

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parent.parent))

from common.io import read_pcd, write_pcd, write_trajectory_tum
from common.transform import (
    euler_to_matrix,
    make_homogeneous,
    matrix_to_quaternion,
    quaternion_to_matrix,
)


def find_map_file(slam_dir: Path) -> Path:
    """Find the global map PCD file from SLAM outputs."""
    candidates = ["scans.pcd", "GlobalMap.pcd", "map.pcd"]
    for name in candidates:
        path = slam_dir / name
        if path.exists():
            return path
    return None


def find_trajectory_file(slam_dir: Path) -> Path:
    """Find the trajectory file from SLAM outputs."""
    candidates = ["pos_log.txt", "trajectory.txt", "path.txt"]
    for name in candidates:
        path = slam_dir / name
        if path.exists():
            return path
    return None


def find_frames_dir(slam_dir: Path) -> Path:
    """Find the per-frame scans directory."""
    candidates = ["scans", "frames", "pcd"]
    for name in candidates:
        path = slam_dir / name
        if path.is_dir():
            return path
    return None


def parse_trajectory(traj_path: Path) -> np.ndarray:
    """Parse trajectory file to TUM format (N,8): [timestamp tx ty tz qx qy qz qw].

    Handles two formats:
      - 8-column TUM: timestamp tx ty tz qx qy qz qw
      - 7-column pos_log: timestamp x y z roll pitch yaw
    """
    rows = []
    with open(traj_path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) >= 8:
                # TUM format: timestamp tx ty tz qx qy qz qw
                rows.append([float(x) for x in parts[:8]])
            elif len(parts) >= 7:
                # pos_log format: timestamp x y z roll pitch yaw
                ts = float(parts[0])
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                roll, pitch, yaw = float(parts[4]), float(parts[5]), float(parts[6])
                R = euler_to_matrix(roll, pitch, yaw)
                qx, qy, qz, qw = matrix_to_quaternion(R)
                rows.append([ts, x, y, z, qx, qy, qz, qw])

    if not rows:
        raise ValueError(f"No valid poses found in {traj_path}")

    return np.array(rows, dtype=np.float64)


def pose_to_transform(pose_row: np.ndarray) -> np.ndarray:
    """Convert a TUM pose row [ts tx ty tz qx qy qz qw] to 4x4 transform."""
    tx, ty, tz = pose_row[1], pose_row[2], pose_row[3]
    qx, qy, qz, qw = pose_row[4], pose_row[5], pose_row[6], pose_row[7]
    R = quaternion_to_matrix(qx, qy, qz, qw)
    return make_homogeneous(R, np.array([tx, ty, tz]))


def transform_cloud(points: np.ndarray, T: np.ndarray) -> np.ndarray:
    """Transform point cloud (N,3) or (N,4) by 4x4 homogeneous transform."""
    xyz = points[:, :3]
    ones = np.ones((xyz.shape[0], 1), dtype=xyz.dtype)
    xyz_h = np.hstack([xyz, ones])  # (N, 4)
    xyz_transformed = (T @ xyz_h.T).T[:, :3]

    if points.shape[1] == 4:
        return np.hstack([xyz_transformed, points[:, 3:4]])
    return xyz_transformed.astype(np.float32)


def export_frames(frames_dir: Path, poses: np.ndarray, output_frames_dir: Path) -> int:
    """Transform per-frame clouds to global coordinates and export.

    Matches frames to poses by sorted order (frame index = pose index).
    Returns number of frames exported.
    """
    output_frames_dir.mkdir(parents=True, exist_ok=True)

    pcd_files = sorted(frames_dir.glob("*.pcd"))
    if not pcd_files:
        print(f"  No PCD files found in {frames_dir}")
        return 0

    num_exported = 0
    num_poses = len(poses)

    for i, pcd_file in enumerate(pcd_files):
        if i >= num_poses:
            print(f"  Warning: more frames ({len(pcd_files)}) than poses ({num_poses}), stopping.")
            break

        points = read_pcd(pcd_file)
        T = pose_to_transform(poses[i])
        points_global = transform_cloud(points, T)

        out_path = output_frames_dir / pcd_file.name
        write_pcd(out_path, points_global)
        num_exported += 1

        if (num_exported) % 100 == 0:
            print(f"  Exported {num_exported}/{min(len(pcd_files), num_poses)} frames")

    return num_exported


def main():
    parser = argparse.ArgumentParser(
        description="Export FAST-LIO2 outputs to standardized formats"
    )
    parser.add_argument(
        "slam_dir", type=Path,
        help="Directory containing FAST-LIO2 outputs"
    )
    parser.add_argument(
        "--output-dir", type=Path, default=None,
        help="Output directory (default: same as slam_dir)"
    )
    parser.add_argument(
        "--skip-frames", action="store_true",
        help="Skip per-frame export (faster if only map/trajectory needed)"
    )

    args = parser.parse_args()

    slam_dir = args.slam_dir.resolve()
    output_dir = (args.output_dir or args.slam_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    if not slam_dir.exists():
        print(f"Error: SLAM directory not found: {slam_dir}")
        sys.exit(1)

    # --- Export global map ---
    map_file = find_map_file(slam_dir)
    if map_file:
        dst = output_dir / "global_map.pcd"
        if map_file.resolve() != dst.resolve():
            import shutil
            shutil.copy2(map_file, dst)
        print(f"Global map: {dst} (from {map_file.name})")
    else:
        print("Warning: No global map file found (looked for scans.pcd, GlobalMap.pcd, map.pcd)")

    # --- Export trajectory ---
    traj_file = find_trajectory_file(slam_dir)
    if traj_file:
        poses = parse_trajectory(traj_file)
        traj_out = output_dir / "trajectory.txt"
        write_trajectory_tum(traj_out, poses)
        print(f"Trajectory: {traj_out} ({len(poses)} poses, from {traj_file.name})")
    else:
        print("Error: No trajectory file found (looked for pos_log.txt, trajectory.txt, path.txt)")
        sys.exit(1)

    # --- Export per-frame clouds ---
    if not args.skip_frames:
        frames_dir = find_frames_dir(slam_dir)
        if frames_dir:
            output_frames_dir = output_dir / "frames"
            num = export_frames(frames_dir, poses, output_frames_dir)
            print(f"Frames: exported {num} clouds to {output_frames_dir}")
        else:
            print("Note: No per-frame scans directory found (looked for scans/, frames/, pcd/)")

    print("Export complete.")


if __name__ == "__main__":
    main()
