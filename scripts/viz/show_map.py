#!/usr/bin/env python3
"""Visualize a PCD point cloud map with height or intensity coloring.

Usage:
    python viz/show_map.py global_map.pcd [--color-by height|intensity] [--voxel-size 0.1] [--save map.png]
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import open3d as o3d

sys.path.insert(0, str(Path(__file__).parent.parent))

from common.io import read_pcd


def color_by_height(points: np.ndarray) -> np.ndarray:
    """Color points by z-coordinate: blue (low) to red (high)."""
    z = points[:, 2]
    z_min, z_max = z.min(), z.max()
    if z_max - z_min < 1e-6:
        t = np.zeros(len(z))
    else:
        t = (z - z_min) / (z_max - z_min)

    colors = np.zeros((len(t), 3))
    colors[:, 0] = t          # R: increases with height
    colors[:, 2] = 1.0 - t    # B: decreases with height
    return colors


def color_by_intensity(points: np.ndarray) -> np.ndarray:
    """Color points by intensity (grayscale). Requires 4-column input."""
    if points.shape[1] < 4:
        print("Warning: No intensity channel found, using uniform gray.")
        return np.full((len(points), 3), 0.5)

    intensity = points[:, 3]
    i_min, i_max = intensity.min(), intensity.max()
    if i_max - i_min < 1e-6:
        t = np.full(len(intensity), 0.5)
    else:
        t = (intensity - i_min) / (i_max - i_min)

    colors = np.column_stack([t, t, t])
    return colors


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
        description="Visualize a PCD point cloud map with coloring options"
    )
    parser.add_argument("map_pcd", type=Path, help="Path to PCD point cloud file")
    parser.add_argument("--color-by", choices=["height", "intensity"], default="height",
                        help="Coloring mode (default: height)")
    parser.add_argument("--voxel-size", type=float, default=None,
                        help="Voxel size for downsampling (meters). None = no downsampling")
    parser.add_argument("--save", type=Path, default=None,
                        help="Save rendering to image (headless mode)")

    args = parser.parse_args()

    if not args.map_pcd.exists():
        print(f"Error: PCD file not found: {args.map_pcd}")
        sys.exit(1)

    print(f"Loading map: {args.map_pcd}")
    points = read_pcd(args.map_pcd)
    print(f"  Points: {len(points)}")

    # Color the point cloud
    if args.color_by == "intensity":
        colors = color_by_intensity(points)
    else:
        colors = color_by_height(points)

    # Create Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3].astype(np.float64))
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # Optional voxel downsampling
    if args.voxel_size is not None and args.voxel_size > 0:
        pcd = pcd.voxel_down_sample(args.voxel_size)
        print(f"  After voxel downsample ({args.voxel_size}m): {len(pcd.points)} points")

    geometries = [pcd]

    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        render_headless(geometries, args.save)
    else:
        o3d.visualization.draw_geometries(
            geometries, window_name="Map Viewer",
            width=1920, height=1080
        )


if __name__ == "__main__":
    main()
