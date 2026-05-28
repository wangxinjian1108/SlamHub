#!/usr/bin/env python3
"""
Register secondary LiDAR point clouds against the primary SLAM map.

Supports two modes:
  - frame: register each secondary frame against a local submap
  - global: accumulate all secondary clouds and register against the full map

Usage:
    python 04_register_secondary.py \
        --primary-map output/slam/global_map.pcd \
        --trajectory output/slam/trajectory.txt \
        --secondary-dir /path/to/raw_pointclouds/flash_front_pointcloud/ \
        --initial-guess /path/to/application.yaml \
        --method icp \
        --mode frame \
        --submap-radius 50.0 \
        --output-dir output/registration/flash_front/
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parent.parent))

from common.io import read_pcd, write_pcd, read_trajectory_tum, read_yaml, write_yaml
from common.transform import (
    euler_to_matrix,
    make_homogeneous,
    quaternion_to_matrix,
    matrix_to_quaternion,
    invert_transform,
)
from registration import get_registration_method

# Mapping from secondary directory name to calibration frame name
LIDAR_FRAME_MAP = {
    "remote_front_left_pointcloud": "FRAME_LIDAR_REMOTE_FRONT_LEFT",
    "remote_front_right_pointcloud": "FRAME_LIDAR_REMOTE_FRONT_RIGHT",
    "flash_front_pointcloud": "FRAME_LIDAR_FLASH_FRONT",
    "flash_rear_pointcloud": "FRAME_LIDAR_FLASH_REAR",
}


def load_initial_guess(yaml_path: Path, secondary_name: str) -> np.ndarray:
    """Load initial extrinsic guess from application.yaml.

    Returns 4x4 homogeneous transform (secondary LiDAR -> base_link).
    """
    frame_name = LIDAR_FRAME_MAP.get(secondary_name)
    if frame_name is None:
        print(f"Warning: No frame mapping for '{secondary_name}', using identity.")
        return np.eye(4)

    config = read_yaml(yaml_path)
    calibrations = config["vehicle"]["calibration"]["sensor_calibration"]

    for cal in calibrations:
        if cal["source"] == frame_name:
            t = cal["transformation"]
            # transformation is [x, y, z, roll, pitch, yaw]
            x, y, z = t[0], t[1], t[2]
            roll, pitch, yaw = t[3], t[4], t[5]
            R = euler_to_matrix(roll, pitch, yaw)
            return make_homogeneous(R, np.array([x, y, z]))

    print(f"Warning: Frame '{frame_name}' not found in calibration, using identity.")
    return np.eye(4)


def pose_to_transform(pose_row: np.ndarray) -> np.ndarray:
    """Convert TUM pose row [ts tx ty tz qx qy qz qw] to 4x4 transform."""
    tx, ty, tz = pose_row[1], pose_row[2], pose_row[3]
    qx, qy, qz, qw = pose_row[4], pose_row[5], pose_row[6], pose_row[7]
    R = quaternion_to_matrix(qx, qy, qz, qw)
    return make_homogeneous(R, np.array([tx, ty, tz]))


def find_closest_pose(timestamp_ns: int, poses: np.ndarray) -> int:
    """Find the index of the closest pose by timestamp.

    Args:
        timestamp_ns: Timestamp in nanoseconds (from PCD filename).
        poses: (N,8) TUM trajectory array with timestamps in seconds.

    Returns:
        Index of the closest pose.
    """
    timestamp_s = timestamp_ns / 1e9
    diffs = np.abs(poses[:, 0] - timestamp_s)
    return int(np.argmin(diffs))


def extract_submap(map_points: np.ndarray, center: np.ndarray, radius: float) -> np.ndarray:
    """Extract points within radius of center from the map.

    Args:
        map_points: (N,3) or (N,4) full map point cloud.
        center: (3,) center position.
        radius: Extraction radius in meters.

    Returns:
        Subset of map_points within the sphere.
    """
    dists = np.linalg.norm(map_points[:, :3] - center.reshape(1, 3), axis=1)
    mask = dists <= radius
    return map_points[mask]


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Downsample point cloud using voxel grid filter.

    Args:
        points: (N,3) or (N,4) point cloud.
        voxel_size: Voxel edge length in meters.

    Returns:
        Downsampled point cloud.
    """
    if len(points) == 0:
        return points

    xyz = points[:, :3]
    # Quantize to voxel grid
    voxel_indices = np.floor(xyz / voxel_size).astype(np.int32)

    # Use unique voxels - keep first point in each voxel
    _, unique_idx = np.unique(voxel_indices, axis=0, return_index=True)
    return points[np.sort(unique_idx)]


def filter_nan_points(points: np.ndarray) -> np.ndarray:
    """Remove points with NaN coordinates."""
    valid_mask = ~np.isnan(points[:, 0])
    return points[valid_mask]


def register_frame_mode(
    map_points: np.ndarray,
    poses: np.ndarray,
    secondary_dir: Path,
    initial_guess: np.ndarray,
    method_name: str,
    submap_radius: float,
    output_dir: Path,
) -> dict:
    """Register each secondary frame against a local submap.

    Returns summary statistics.
    """
    pcd_files = sorted(secondary_dir.glob("*.pcd"))
    if not pcd_files:
        print("Error: No PCD files found in secondary directory.")
        sys.exit(1)

    print(f"Frame mode: registering {len(pcd_files)} secondary frames")
    print(f"  Method: {method_name}, Submap radius: {submap_radius}m")

    reg_method = get_registration_method(method_name)
    transforms_file = output_dir / "frame_transforms.txt"
    output_dir.mkdir(parents=True, exist_ok=True)

    fitness_values = []
    rmse_values = []
    t_start = time.time()

    with open(transforms_file, "w") as f_out:
        f_out.write("# timestamp T00 T01 T02 T03 T10 T11 T12 T13 T20 T21 T22 T23\n")

        for i, pcd_file in enumerate(pcd_files):
            # Parse timestamp from filename (nanoseconds)
            timestamp_ns = int(pcd_file.stem)

            # Load and filter secondary cloud
            sec_points = read_pcd(pcd_file)
            sec_points = filter_nan_points(sec_points)

            if len(sec_points) < 10:
                continue

            # Find closest primary pose
            pose_idx = find_closest_pose(timestamp_ns, poses)
            T_world_base = pose_to_transform(poses[pose_idx])
            center = T_world_base[:3, 3]

            # Extract local submap
            submap = extract_submap(map_points, center, submap_radius)
            if len(submap) < 10:
                print(f"  Warning: submap too small at frame {i}, skipping.")
                continue

            # Initial guess: T_world_secondary = T_world_base @ T_base_secondary
            T_init = T_world_base @ initial_guess

            # Register secondary cloud against submap
            result = reg_method.register(
                source=sec_points[:, :3],
                target=submap[:, :3],
                initial_guess=T_init,
            )

            # Write 3x4 transform row
            T = result.transformation
            row = [timestamp_ns] + T[:3, :4].flatten().tolist()
            f_out.write(" ".join(f"{v:.8f}" if idx > 0 else str(int(v))
                                 for idx, v in enumerate(row)) + "\n")

            fitness_values.append(result.fitness)
            rmse_values.append(result.inlier_rmse)

            if (i + 1) % 50 == 0:
                print(f"  Processed {i + 1}/{len(pcd_files)} frames "
                      f"(avg fitness: {np.mean(fitness_values):.4f})")

    elapsed = time.time() - t_start
    summary = {
        "mode": "frame",
        "method": method_name,
        "num_frames": len(pcd_files),
        "num_registered": len(fitness_values),
        "submap_radius": submap_radius,
        "mean_fitness": float(np.mean(fitness_values)) if fitness_values else 0.0,
        "mean_rmse": float(np.mean(rmse_values)) if rmse_values else 0.0,
        "std_fitness": float(np.std(fitness_values)) if fitness_values else 0.0,
        "elapsed_seconds": round(elapsed, 2),
    }

    print(f"  Done: {len(fitness_values)}/{len(pcd_files)} frames registered "
          f"in {elapsed:.1f}s")
    return summary


def register_global_mode(
    map_points: np.ndarray,
    poses: np.ndarray,
    secondary_dir: Path,
    initial_guess: np.ndarray,
    method_name: str,
    output_dir: Path,
) -> dict:
    """Accumulate all secondary clouds and register against the full map.

    Returns summary statistics.
    """
    pcd_files = sorted(secondary_dir.glob("*.pcd"))
    if not pcd_files:
        print("Error: No PCD files found in secondary directory.")
        sys.exit(1)

    print(f"Global mode: accumulating {len(pcd_files)} secondary frames")
    print(f"  Method: {method_name}")

    # Accumulate all secondary clouds in world frame using primary poses + initial guess
    accumulated = []
    for i, pcd_file in enumerate(pcd_files):
        timestamp_ns = int(pcd_file.stem)
        sec_points = read_pcd(pcd_file)
        sec_points = filter_nan_points(sec_points)

        if len(sec_points) < 10:
            continue

        # Transform to world: T_world_base @ T_base_secondary
        pose_idx = find_closest_pose(timestamp_ns, poses)
        T_world_base = pose_to_transform(poses[pose_idx])
        T_world_sec = T_world_base @ initial_guess

        # Transform points to world frame
        xyz = sec_points[:, :3]
        ones = np.ones((xyz.shape[0], 1), dtype=xyz.dtype)
        xyz_h = np.hstack([xyz, ones])
        xyz_world = (T_world_sec @ xyz_h.T).T[:, :3]
        accumulated.append(xyz_world)

        if (i + 1) % 100 == 0:
            print(f"  Accumulated {i + 1}/{len(pcd_files)} frames")

    if not accumulated:
        print("Error: No valid secondary frames to accumulate.")
        sys.exit(1)

    all_points = np.vstack(accumulated).astype(np.float32)
    print(f"  Total accumulated points: {len(all_points)}")

    # Downsample accumulated cloud
    all_points_ds = voxel_downsample(all_points, voxel_size=0.2)
    print(f"  After voxel downsample (0.2m): {len(all_points_ds)} points")

    # Register against full map
    reg_method = get_registration_method(method_name)
    t_start = time.time()

    result = reg_method.register(
        source=all_points_ds,
        target=map_points[:, :3],
        initial_guess=np.eye(4),  # Already in world frame from accumulation
    )

    elapsed = time.time() - t_start
    output_dir.mkdir(parents=True, exist_ok=True)

    # Write the single global transform
    transforms_file = output_dir / "frame_transforms.txt"
    with open(transforms_file, "w") as f_out:
        f_out.write("# Global registration result (3x4 transform)\n")
        T = result.transformation
        row = T[:3, :4].flatten().tolist()
        f_out.write(" ".join(f"{v:.8f}" for v in row) + "\n")

    summary = {
        "mode": "global",
        "method": method_name,
        "num_frames_accumulated": len(pcd_files),
        "total_points": int(len(all_points)),
        "downsampled_points": int(len(all_points_ds)),
        "fitness": float(result.fitness),
        "inlier_rmse": float(result.inlier_rmse),
        "num_inliers": int(result.num_inliers),
        "elapsed_seconds": round(elapsed, 2),
    }

    print(f"  Global registration: fitness={result.fitness:.4f}, "
          f"RMSE={result.inlier_rmse:.4f}, inliers={result.num_inliers}")
    return summary


def main():
    parser = argparse.ArgumentParser(
        description="Register secondary LiDAR point clouds against primary SLAM map"
    )
    parser.add_argument(
        "--primary-map", type=Path, required=True,
        help="Path to primary SLAM global map PCD"
    )
    parser.add_argument(
        "--trajectory", type=Path, required=True,
        help="Path to primary SLAM trajectory (TUM format)"
    )
    parser.add_argument(
        "--secondary-dir", type=Path, required=True,
        help="Directory containing secondary LiDAR PCD files (named by timestamp_ns)"
    )
    parser.add_argument(
        "--initial-guess", type=Path, default=None,
        help="Path to application.yaml with extrinsic calibration"
    )
    parser.add_argument(
        "--method", type=str, default="icp",
        help="Registration method (default: icp)"
    )
    parser.add_argument(
        "--mode", type=str, choices=["frame", "global"], default="frame",
        help="Registration mode: 'frame' or 'global' (default: frame)"
    )
    parser.add_argument(
        "--submap-radius", type=float, default=50.0,
        help="Radius for local submap extraction in frame mode (default: 50.0m)"
    )
    parser.add_argument(
        "--output-dir", type=Path, default=Path("output/registration"),
        help="Output directory (default: output/registration/)"
    )

    args = parser.parse_args()

    # Validate inputs
    if not args.primary_map.exists():
        print(f"Error: Primary map not found: {args.primary_map}")
        sys.exit(1)
    if not args.trajectory.exists():
        print(f"Error: Trajectory not found: {args.trajectory}")
        sys.exit(1)
    if not args.secondary_dir.exists():
        print(f"Error: Secondary directory not found: {args.secondary_dir}")
        sys.exit(1)

    # Load primary map and trajectory
    print(f"Loading primary map: {args.primary_map}")
    map_points = read_pcd(args.primary_map)
    print(f"  Map points: {len(map_points)}")

    print(f"Loading trajectory: {args.trajectory}")
    poses = read_trajectory_tum(args.trajectory)
    print(f"  Poses: {len(poses)}")

    # Load initial extrinsic guess
    if args.initial_guess and args.initial_guess.exists():
        secondary_name = args.secondary_dir.name
        initial_guess = load_initial_guess(args.initial_guess, secondary_name)
        print(f"Initial guess loaded for '{secondary_name}'")
    else:
        initial_guess = np.eye(4)
        if args.initial_guess:
            print(f"Warning: Initial guess file not found: {args.initial_guess}, using identity.")
        else:
            print("No initial guess provided, using identity transform.")

    # Run registration
    if args.mode == "frame":
        summary = register_frame_mode(
            map_points=map_points,
            poses=poses,
            secondary_dir=args.secondary_dir,
            initial_guess=initial_guess,
            method_name=args.method,
            submap_radius=args.submap_radius,
            output_dir=args.output_dir,
        )
    else:
        summary = register_global_mode(
            map_points=map_points,
            poses=poses,
            secondary_dir=args.secondary_dir,
            initial_guess=initial_guess,
            method_name=args.method,
            output_dir=args.output_dir,
        )

    # Write summary
    summary_path = args.output_dir / "summary.yaml"
    write_yaml(summary_path, summary)
    print(f"Summary written to: {summary_path}")
    print("Done.")


if __name__ == "__main__":
    main()
