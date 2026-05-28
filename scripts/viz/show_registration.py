#!/usr/bin/env python3
"""Visualize registration overlay: primary map vs transformed secondary frame.

Usage:
    python viz/show_registration.py global_map.pcd \
        --secondary output/registration/flash_front/ \
        --secondary-pcd-dir /path/to/raw_pcds/ \
        --frame 100 [--save reg.png] [--voxel-size 0.1]
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import open3d as o3d

sys.path.insert(0, str(Path(__file__).parent.parent))

from common.io import read_pcd


def load_frame_transform(transforms_file: Path, frame_idx: int) -> np.ndarray:
    """Load the transform for a specific frame index from frame_transforms.txt.

    Returns 4x4 homogeneous transform.
    """
    transforms = []
    with open(transforms_file, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            transforms.append(line)

    if frame_idx >= len(transforms):
        print(f"Error: Frame index {frame_idx} out of range "
              f"(only {len(transforms)} transforms available).")
        sys.exit(1)

    parts = transforms[frame_idx].split()
    # Format: timestamp T00 T01 T02 T03 T10 T11 T12 T13 T20 T21 T22 T23
    values = [float(x) for x in parts[1:]]  # skip timestamp
    T = np.eye(4)
    T[:3, :4] = np.array(values).reshape(3, 4)
    return T


def get_secondary_pcd_path(pcd_dir: Path, transforms_file: Path, frame_idx: int) -> Path:
    """Get the PCD file path for the given frame index using the timestamp."""
    transforms = []
    with open(transforms_file, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            transforms.append(line)

    if frame_idx >= len(transforms):
        print(f"Error: Frame index {frame_idx} out of range.")
        sys.exit(1)

    parts = transforms[frame_idx].split()
    timestamp = parts[0]  # nanosecond timestamp
    pcd_path = pcd_dir / f"{timestamp}.pcd"
    return pcd_path


def render_headless(geometries: list, save_path: Path) -> None:
    """Render geometries offscreen and save to image."""
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False, width=1920, height=1080)
    for geom in geometries:
        vis.add_geometry(geom)
    vis.poll_events()
    vis.update_renderer()
    vis.capture_screen_image(str(save_path))
    vis.destroy_window()
    print(f"Saved: {save_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Visualize registration overlay of primary map and secondary frame"
    )
    parser.add_argument("map_pcd", type=Path, help="Path to primary map PCD file")
    parser.add_argument("--secondary", type=Path, required=True,
                        help="Registration output directory (contains frame_transforms.txt)")
    parser.add_argument("--secondary-pcd-dir", type=Path, required=True,
                        help="Directory containing raw secondary PCD files")
    parser.add_argument("--frame", type=int, default=0,
                        help="Frame index to visualize (default: 0)")
    parser.add_argument("--voxel-size", type=float, default=None,
                        help="Voxel size for downsampling (meters)")
    parser.add_argument("--save", type=Path, default=None,
                        help="Save rendering to image (headless mode)")

    args = parser.parse_args()

    if not args.map_pcd.exists():
        print(f"Error: Map PCD not found: {args.map_pcd}")
        sys.exit(1)

    transforms_file = args.secondary / "frame_transforms.txt"
    if not transforms_file.exists():
        print(f"Error: Transforms file not found: {transforms_file}")
        sys.exit(1)

    if not args.secondary_pcd_dir.exists():
        print(f"Error: Secondary PCD directory not found: {args.secondary_pcd_dir}")
        sys.exit(1)

    # Load primary map
    print(f"Loading primary map: {args.map_pcd}")
    map_points = read_pcd(args.map_pcd)
    print(f"  Map points: {len(map_points)}")

    # Load secondary frame
    sec_pcd_path = get_secondary_pcd_path(
        args.secondary_pcd_dir, transforms_file, args.frame
    )
    if not sec_pcd_path.exists():
        print(f"Error: Secondary PCD not found: {sec_pcd_path}")
        sys.exit(1)

    print(f"Loading secondary frame: {sec_pcd_path}")
    sec_points = read_pcd(sec_pcd_path)
    print(f"  Secondary points: {len(sec_points)}")

    # Load transform for this frame
    T = load_frame_transform(transforms_file, args.frame)

    # Create primary point cloud (red)
    pcd_primary = o3d.geometry.PointCloud()
    pcd_primary.points = o3d.utility.Vector3dVector(
        map_points[:, :3].astype(np.float64)
    )
    pcd_primary.paint_uniform_color([1.0, 0.3, 0.3])

    # Create secondary point cloud, apply transform (blue)
    pcd_secondary = o3d.geometry.PointCloud()
    pcd_secondary.points = o3d.utility.Vector3dVector(
        sec_points[:, :3].astype(np.float64)
    )
    pcd_secondary.transform(T)
    pcd_secondary.paint_uniform_color([0.3, 0.3, 1.0])

    # Optional voxel downsampling
    if args.voxel_size is not None and args.voxel_size > 0:
        pcd_primary = pcd_primary.voxel_down_sample(args.voxel_size)
        pcd_secondary = pcd_secondary.voxel_down_sample(args.voxel_size)
        print(f"  After voxel downsample ({args.voxel_size}m): "
              f"primary={len(pcd_primary.points)}, "
              f"secondary={len(pcd_secondary.points)}")

    geometries = [pcd_primary, pcd_secondary]

    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        render_headless(geometries, args.save)
    else:
        o3d.visualization.draw_geometries(
            geometries, window_name="Registration Viewer",
            width=1920, height=1080
        )


if __name__ == "__main__":
    main()
