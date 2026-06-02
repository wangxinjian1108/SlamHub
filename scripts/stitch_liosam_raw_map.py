#!/usr/bin/env python3
"""Stitch a raw-PCD full map using LIO-SAM's trajectory.

§17.7.2 follow-up: LIO-SAM's GlobalMap.pcd is feature-only (corner+surf,
4.25M pts) and that hurts cross-LiDAR ICP on the lateral axes. This
script rebuilds a full raw-PCD map by reusing the cleaned PCDs from the
KISS-ICP run + the LIO-SAM trajectory_lidar.txt poses.

Output:
    <output-dir>/scans.pcd                  full raw map
    <output-dir>/scans_voxel0.3.pcd         voxel-0.3 (B2 ICP target)
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from common.io import write_pcd, read_trajectory_tum
from common.transform import quaternion_to_matrix, make_homogeneous
from convert_to_rosbag_velodyne import parse_pcd_binary


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--cleaned-pcd-dir", type=Path,
                   default=Path("output/kiss_icp_run/cleaned_pcds"))
    p.add_argument("--trajectory-lidar", type=Path,
                   default=Path("output/liosam_run/trajectory_lidar.txt"))
    p.add_argument("--output-dir", type=Path,
                   default=Path("output/liosam_run_hybrid"))
    p.add_argument("--voxel", type=float, default=0.3)
    args = p.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)

    pcd_files = sorted(args.cleaned_pcd_dir.glob("*.pcd"))
    traj = read_trajectory_tum(args.trajectory_lidar)
    print(f"  cleaned PCDs: {len(pcd_files)}")
    print(f"  trajectory:   {len(traj)} poses")

    # The cleaned PCD filenames are timestamps (ns); the trajectory has
    # timestamps (s). Sanity-check: the i-th pcd should match the i-th pose
    # since both were sourced from the same primary LiDAR's PCD timestamps.
    n = min(len(pcd_files), len(traj))
    print(f"  stitching {n} frames...")

    all_pts = []
    for i in range(n):
        pts = parse_pcd_binary(pcd_files[i])
        xyz = np.column_stack([pts["x"], pts["y"], pts["z"]]).astype(np.float32)
        valid = ~np.isnan(xyz).any(axis=1)
        xyz = xyz[valid]
        ts, tx, ty, tz, qx, qy, qz, qw = traj[i]
        # Sanity: frame timestamp should match within a few ms
        pcd_ts = int(pcd_files[i].stem) / 1e9
        if abs(pcd_ts - ts) > 0.05:
            print(f"  WARN: frame {i} pcd_ts={pcd_ts:.3f} vs traj_ts={ts:.3f} (Δ={pcd_ts-ts:.3f}s)")
        R = quaternion_to_matrix(qx, qy, qz, qw)
        T = make_homogeneous(R, np.array([tx, ty, tz]))
        h = np.hstack([xyz, np.ones((len(xyz), 1), dtype=np.float32)])
        all_pts.append((T @ h.T).T[:, :3].astype(np.float32))
        if (i + 1) % 100 == 0:
            print(f"    {i+1}/{n}  ({sum(len(a) for a in all_pts):,} pts so far)")

    pts = np.vstack(all_pts).astype(np.float32)
    print(f"  total: {len(pts):,} points")

    # Write full map (large)
    out_full = args.output_dir / "scans.pcd"
    write_pcd(out_full, pts)
    print(f"  wrote {out_full}  ({out_full.stat().st_size/1e6:.1f} MB)")

    # Voxel downsample for B2 target
    import open3d as o3d
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
    ds = pcd.voxel_down_sample(args.voxel)
    out_voxel = args.output_dir / f"scans_voxel{args.voxel}.pcd"
    o3d.io.write_point_cloud(str(out_voxel), ds)
    print(f"  voxel {args.voxel} m: {len(pts):,} → {len(ds.points):,} pts")
    print(f"  wrote {out_voxel}  ({out_voxel.stat().st_size/1e6:.1f} MB)")


if __name__ == "__main__":
    main()
