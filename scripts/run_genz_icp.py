#!/usr/bin/env python3
"""Run GenZ-ICP backend end-to-end on a recording directory.

GenZ-ICP (RA-L 2025) is degeneracy-robust LiDAR odometry from POSTECH —
KISS-ICP-style pipeline (pure Python via `pip install genz-icp`) with an
adaptive weighting scheme that biases toward planar features in degenerate
geometries (corridors, tunnels). On well-conditioned outdoor data it
should match KISS-ICP; in degenerate scenes it should outperform.

Pipeline (mirrors run_kiss_icp.py):
  1. Reuse cleaned PCDs from KISS-ICP run (NaN-stripped, 3-field xyz).
     genz-icp's PCD loader needs the same input format.
  2. Run `genz_icp_pipeline <cleaned-pcd-dir>`
  3. Inject real timestamps from PCD filenames into the TUM trajectory
     (genz-icp emits frame-index timestamps 0,1,2,...)
  4. Compose to baselink: T_world_baselink = T_world_lidar @ T_lidar_baselink
  5. Compose baselink → IMU: T_world_imu = T_world_baselink @ T_baselink_imu
     (so the existing B2 pipeline that assumes T_world_imu input works)
  6. Stitch a global map from the per-frame poses + cleaned PCDs,
     voxel-downsample to 0.3 m

Outputs in --output-dir:
  trajectory_lidar.txt    T_world_lidar (LiDAR frame), TUM
  trajectory_baselink.txt T_world_baselink, TUM
  trajectory.txt          T_world_imu (B2-compatible), TUM
  scans.pcd               full accumulated map
  scans_voxel0.3.pcd      voxel-downsampled (B2 ICP target)
  genz_raw/               GenZ-ICP raw output (config.yml + npy + kitti txt)
"""
import argparse
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).parent))
from common.io import write_pcd
from common.transform import (
    euler_to_matrix, quaternion_to_matrix, matrix_to_quaternion,
    make_homogeneous, invert_transform,
)
from convert_to_rosbag_velodyne import parse_pcd_binary
from run_kiss_icp import (
    LIDAR_FRAME, load_extrinsic, clean_pcds, inject_timestamps,
    compose_to_baselink, compose_baselink_to_imu, write_tum, stitch_map,
)


def run_genz(cleaned_dir, raw_out_dir):
    """Invoke genz_icp_pipeline; return path to TUM poses file."""
    raw_out_dir.mkdir(parents=True, exist_ok=True)
    print(f"  running genz_icp_pipeline on {cleaned_dir}...")
    res = subprocess.run(
        # Pass cleaned_dir as absolute path — we set cwd=raw_out_dir so the
        # results/ tree lands inside it, and a relative cleaned_dir would be
        # resolved against that cwd, breaking the input lookup.
        ["genz_icp_pipeline", str(Path(cleaned_dir).resolve())],
        cwd=str(raw_out_dir),
        capture_output=True, text=True,
    )
    if res.returncode != 0:
        print(res.stdout); print(res.stderr, file=sys.stderr)
        sys.exit(f"  genz_icp_pipeline failed (exit {res.returncode})")
    # GenZ-ICP writes results/<timestamp>/<basename>_poses_tum.txt
    results = sorted((raw_out_dir / "results").glob("*"))
    if not results:
        sys.exit(f"  no results/ produced under {raw_out_dir}")
    latest = results[-1]
    tum_files = list(latest.glob("*_poses_tum.txt"))
    if not tum_files:
        sys.exit(f"  no _poses_tum.txt in {latest}")
    print(f"  genz-icp OK, poses at {tum_files[0]}")
    return tum_files[0]


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("recording_dir", type=Path)
    p.add_argument("--primary-lidar", default="remote_front_left_pointcloud")
    p.add_argument("--output-dir", type=Path, required=True)
    p.add_argument("--voxel-size", type=float, default=0.3)
    p.add_argument("--cleaned-pcd-dir", type=Path, default=None,
                   help="Reuse an existing cleaned-PCDs dir (e.g. from a "
                        "prior KISS-ICP run); skips re-cleaning.")
    args = p.parse_args()

    rec = args.recording_dir
    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)

    print("=== GenZ-ICP backend ===")
    print(f"  recording: {rec}")
    print(f"  primary:   {args.primary_lidar}")
    print(f"  output:    {out}")

    # 1. clean (or reuse pre-cleaned)
    src_pcd_dir = rec / "raw_pointclouds" / args.primary_lidar
    if args.cleaned_pcd_dir is not None and args.cleaned_pcd_dir.exists():
        cleaned_dir = args.cleaned_pcd_dir
        print(f"  reusing cleaned PCDs: {cleaned_dir}")
    else:
        cleaned_dir = out / "cleaned_pcds"
        clean_pcds(src_pcd_dir, cleaned_dir)

    # 2. genz-icp
    raw_out = out / "genz_raw"
    genz_tum = run_genz(cleaned_dir, raw_out)

    # 3. inject real timestamps
    data_lidar = inject_timestamps(genz_tum, cleaned_dir)
    write_tum(out / "trajectory_lidar.txt", data_lidar,
              "GenZ-ICP T_world_lidar (LiDAR frame)")
    print(f"  trajectory_lidar.txt: {len(data_lidar)} poses")

    # 4. compose to baselink
    frame_name = LIDAR_FRAME.get(args.primary_lidar)
    if not frame_name:
        sys.exit(f"  unknown primary lidar '{args.primary_lidar}'")
    T_baselink_lidar = load_extrinsic(rec / "application.yaml", frame_name)
    if T_baselink_lidar is None:
        sys.exit(f"  could not load {frame_name} extrinsic")
    T_lidar_baselink = invert_transform(T_baselink_lidar)
    data_baselink = compose_to_baselink(data_lidar, T_lidar_baselink)
    write_tum(out / "trajectory_baselink.txt", data_baselink,
              "GenZ-ICP T_world_baselink")

    # 5. compose to fake IMU (for B2 compat)
    T_baselink_imu = load_extrinsic(rec / "application.yaml", "FRAME_GNSS_IMU")
    if T_baselink_imu is None:
        print("  warning: FRAME_GNSS_IMU not found; trajectory.txt == baselink")
        T_baselink_imu = np.eye(4)
    data_imu = compose_baselink_to_imu(data_baselink, T_baselink_imu)
    write_tum(out / "trajectory.txt", data_imu,
              "GenZ-ICP T_world_imu (synthetic, for B2 pipeline compat)")

    # 6. stitch map
    stitch_map(cleaned_dir, data_lidar,
               out / "scans.pcd",
               out / f"scans_voxel{args.voxel_size}.pcd",
               voxel=args.voxel_size)

    print("=== Done ===")
    print(f"  trajectory.txt:               {out / 'trajectory.txt'}")
    print(f"  trajectory_baselink.txt:      {out / 'trajectory_baselink.txt'}")
    print(f"  trajectory_lidar.txt:         {out / 'trajectory_lidar.txt'}")
    print(f"  scans.pcd:                    {out / 'scans.pcd'}")
    print(f"  scans_voxel{args.voxel_size}.pcd:           "
          f"{out / f'scans_voxel{args.voxel_size}.pcd'}")


if __name__ == "__main__":
    main()
