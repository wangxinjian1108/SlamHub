#!/usr/bin/env python3
"""Aggregate per-frame registration transforms into a final calibrated extrinsic.

Reads frame_transforms.txt files produced by the registration step (Task 6),
filters outliers, averages the remaining transforms, and outputs a YAML file
with the calibrated extrinsic for each secondary LiDAR.

Usage:
    python 05_solve_extrinsic.py \
        --registration-dir output/registration/ \
        --primary-lidar remote_front_left_pointcloud \
        --output output/calibration/extrinsics.yaml
"""

import argparse
import sys
from pathlib import Path

import numpy as np

# Allow imports from the scripts directory
sys.path.insert(0, str(Path(__file__).parent.parent))

from scripts.common.io import write_yaml
from scripts.common.transform import matrix_to_euler
from scripts.solve_extrinsic_utils import aggregate_transforms, filter_outliers


def parse_frame_transforms(filepath: Path) -> list:
    """Parse a frame_transforms.txt file into a list of 4x4 matrices.

    Each line has 14 values: frame_idx timestamp_ns followed by 12 values
    representing a 3x4 matrix in row-major order.

    Returns:
        List of 4x4 homogeneous transform matrices.
    """
    transforms = []
    with open(filepath, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 14:
                continue
            # Skip frame_idx and timestamp_ns (first 2 values)
            values = [float(x) for x in parts[2:14]]
            # Reshape 12 values into 3x4 matrix (row-major)
            mat34 = np.array(values).reshape(3, 4)
            T = np.eye(4)
            T[:3, :] = mat34
            transforms.append(T)
    return transforms


def main():
    parser = argparse.ArgumentParser(
        description="Aggregate per-frame transforms into calibrated extrinsics."
    )
    parser.add_argument(
        "--registration-dir",
        type=str,
        required=True,
        help="Directory containing per-lidar registration subdirectories.",
    )
    parser.add_argument(
        "--primary-lidar",
        type=str,
        required=True,
        help="Name of the primary LiDAR used for SLAM.",
    )
    parser.add_argument(
        "--output",
        type=str,
        required=True,
        help="Output path for the extrinsics YAML file.",
    )
    parser.add_argument(
        "--outlier-threshold",
        type=float,
        default=3.0,
        help="Outlier filtering threshold in standard deviations (default: 3.0).",
    )
    args = parser.parse_args()

    registration_dir = Path(args.registration_dir)
    output_path = Path(args.output)

    if not registration_dir.is_dir():
        print(f"Error: registration directory not found: {registration_dir}")
        sys.exit(1)

    # Iterate over subdirectories in registration-dir
    secondary_lidars = {}
    subdirs = sorted([d for d in registration_dir.iterdir() if d.is_dir()])

    if not subdirs:
        print(f"Warning: no subdirectories found in {registration_dir}")

    for subdir in subdirs:
        lidar_name = subdir.name
        transforms_file = subdir / "frame_transforms.txt"

        if not transforms_file.exists():
            print(f"Skipping {lidar_name}: no frame_transforms.txt found")
            continue

        # Load transforms
        transforms = parse_frame_transforms(transforms_file)
        num_total = len(transforms)

        if num_total == 0:
            print(f"Skipping {lidar_name}: no valid transforms found")
            continue

        # Filter outliers
        filtered = filter_outliers(transforms, threshold=args.outlier_threshold)
        num_used = len(filtered)

        # Aggregate
        T_mean = aggregate_transforms(filtered)

        # Compute translation std for quality metric
        translations = np.array([T[:3, 3] for T in filtered])
        translation_std = float(np.mean(np.std(translations, axis=0)))

        # Convert to euler for human-readable output
        roll, pitch, yaw = matrix_to_euler(T_mean[:3, :3])
        tx, ty, tz = T_mean[:3, 3].tolist()

        secondary_lidars[lidar_name] = {
            "transform_xyzrpy": [tx, ty, tz, float(roll), float(pitch), float(yaw)],
            "transform_matrix": T_mean.tolist(),
            "num_frames_used": num_used,
            "num_frames_total": num_total,
            "translation_std": translation_std,
        }

        print(
            f"{lidar_name}: used {num_used}/{num_total} frames, "
            f"translation_std={translation_std:.4f}"
        )

    # Write output
    output_data = {
        "primary_lidar": args.primary_lidar,
        "secondary_lidars": secondary_lidars,
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    write_yaml(output_path, output_data)
    print(f"\nExtrinsics written to: {output_path}")


if __name__ == "__main__":
    main()
