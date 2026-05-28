#!/usr/bin/env python3
"""Visualize a TUM-format trajectory as a colored line with coordinate frames.

Usage:
    python viz/show_trajectory.py trajectory.txt [--save traj.png] [--axes-every 20]
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import open3d as o3d

sys.path.insert(0, str(Path(__file__).parent.parent))

from common.io import read_trajectory_tum
from common.transform import quaternion_to_matrix, make_homogeneous


def build_trajectory_lineset(poses: np.ndarray) -> o3d.geometry.LineSet:
    """Create a LineSet from trajectory positions, colored blue-to-red by time."""
    positions = poses[:, 1:4]  # tx, ty, tz
    n = len(positions)

    lines = [[i, i + 1] for i in range(n - 1)]
    colors = []
    for i in range(n - 1):
        t = i / max(n - 2, 1)
        # Blue (0,0,1) -> Red (1,0,0)
        colors.append([t, 0.0, 1.0 - t])

    lineset = o3d.geometry.LineSet()
    lineset.points = o3d.utility.Vector3dVector(positions)
    lineset.lines = o3d.utility.Vector2iVector(lines)
    lineset.colors = o3d.utility.Vector3dVector(colors)
    return lineset


def build_axes_meshes(poses: np.ndarray, every: int, size: float = 0.5) -> list:
    """Create coordinate frame meshes at every N-th pose."""
    meshes = []
    for i in range(0, len(poses), every):
        row = poses[i]
        tx, ty, tz = row[1], row[2], row[3]
        qx, qy, qz, qw = row[4], row[5], row[6], row[7]
        R = quaternion_to_matrix(qx, qy, qz, qw)
        T = make_homogeneous(R, np.array([tx, ty, tz]))

        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
        frame.transform(T)
        meshes.append(frame)
    return meshes


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
        description="Visualize a TUM-format trajectory as a 3D colored line"
    )
    parser.add_argument("trajectory", type=Path, help="Path to TUM trajectory file")
    parser.add_argument("--save", type=Path, default=None,
                        help="Save rendering to image (headless mode)")
    parser.add_argument("--axes-every", type=int, default=20,
                        help="Show coordinate frame every N poses (default: 20)")
    parser.add_argument("--axes-size", type=float, default=0.5,
                        help="Size of coordinate frame meshes (default: 0.5)")

    args = parser.parse_args()

    if not args.trajectory.exists():
        print(f"Error: Trajectory file not found: {args.trajectory}")
        sys.exit(1)

    print(f"Loading trajectory: {args.trajectory}")
    poses = read_trajectory_tum(args.trajectory)
    print(f"  Poses: {len(poses)}")

    if len(poses) < 2:
        print("Error: Need at least 2 poses to draw a trajectory.")
        sys.exit(1)

    # Build geometries
    lineset = build_trajectory_lineset(poses)
    axes = build_axes_meshes(poses, every=args.axes_every, size=args.axes_size)
    geometries = [lineset] + axes

    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        render_headless(geometries, args.save)
    else:
        o3d.visualization.draw_geometries(
            geometries, window_name="Trajectory Viewer",
            width=1920, height=1080
        )


if __name__ == "__main__":
    main()
