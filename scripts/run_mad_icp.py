#!/usr/bin/env python3
"""Run MAD-ICP backend end-to-end on a recording directory.

MAD-ICP (RVP group / Sapienza, RA-L 2024) — "Matching Data" — is a minimal
voxel-tree LiDAR odometry that prioritizes runtime + robustness. Like
KISS-ICP / GenZ-ICP it ships as a pip wheel, but the CLI takes ROS bags or
KITTI-style .bin files (raw float32 N×4: x,y,z,intensity), NOT a PCD
directory. So this wrapper:

  1. Converts cleaned PCDs → KITTI .bin format (float32 N×4)
  2. Generates a custom dataset config (AT128P sensor params, identity
     lidar_to_base since we compose to baselink/IMU separately)
  3. Runs `mad_icp --data-path <bin-dir> --estimate-path <out-dir> ...`
  4. Parses MAD-ICP's KITTI-format estimate.txt (12 floats per line, 3×4
     row-major) and injects real PCD timestamps → TUM trajectory
  5. Composes to baselink → fake IMU (same as run_kiss_icp, B2-compat)
  6. Stitches a global map (raw points × per-frame poses, voxel 0.3)

Output layout matches run_kiss_icp / run_genz_icp:
  trajectory_lidar.txt         T_world_lidar, TUM
  trajectory_baselink.txt      T_world_baselink, TUM
  trajectory.txt               T_world_imu, TUM (B2 input)
  scans.pcd                    full raw map
  scans_voxel0.3.pcd           voxel-0.3 (B2 ICP target)
  mad_raw/                     MAD-ICP's estimate.txt + bin/ + dataset.cfg
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
    quaternion_to_matrix, matrix_to_quaternion,
    make_homogeneous, invert_transform,
)
from convert_to_rosbag_velodyne import parse_pcd_binary
from run_kiss_icp import (
    LIDAR_FRAME, load_extrinsic, clean_pcds,
    compose_to_baselink, compose_baselink_to_imu, write_tum, stitch_map,
)


def pcds_to_kitti_bin(cleaned_dir: Path, bin_dir: Path):
    """Convert PCDs → MAD-ICP KITTI-format .bin (float32 N×4: x,y,z,intensity).

    KittiReader iterates over `*.bin` sorted by natsort; each file is read
    with `np.fromfile(..., dtype=float32).reshape(-1, 4)`. We use the PCD's
    intensity if present, else 0.
    """
    bin_dir.mkdir(parents=True, exist_ok=True)
    pcd_files = sorted(cleaned_dir.glob("*.pcd"))
    print(f"  converting {len(pcd_files)} PCDs to KITTI .bin in {bin_dir}")
    for i, p in enumerate(pcd_files):
        d = parse_pcd_binary(p)
        x = d["x"].astype(np.float32)
        y = d["y"].astype(np.float32)
        z = d["z"].astype(np.float32)
        intensity = (d["intensity"].astype(np.float32) if "intensity" in d.dtype.names
                     else np.zeros_like(x))
        out = np.column_stack([x, y, z, intensity]).astype(np.float32)
        # Use frame index 6-digit as KITTI convention (kitti_reader uses
        # natsorted .bin; any monotonic name works, but we keep the original
        # nanosecond timestamps so stitch_map can re-associate).
        out.tofile(bin_dir / f"{p.stem}.bin")
        if (i + 1) % 200 == 0:
            print(f"    {i+1}/{len(pcd_files)}")
    print(f"  done — {len(pcd_files)} .bin files")


def write_dataset_config(out_path: Path, min_range=1.0, max_range=200.0,
                          sensor_hz=10.0, deskew=False):
    """Write a minimal MAD-ICP dataset config (identity lidar_to_base — we
    compose to baselink ourselves later)."""
    cfg = {
        "min_range": min_range,
        "max_range": max_range,
        "sensor_hz": sensor_hz,
        "deskew": deskew,
        "apply_correction": False,
        "lidar_to_base": [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
    }
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as f:
        yaml.safe_dump(cfg, f, default_flow_style=False)


def run_mad(bin_dir: Path, dataset_cfg: Path, mad_out_dir: Path,
            num_cores=4, num_keyframes=4):
    """Invoke `mad_icp` CLI; return path to estimate.txt."""
    mad_out_dir.mkdir(parents=True, exist_ok=True)
    print(f"  running mad_icp on {bin_dir}...")
    res = subprocess.run(
        [
            "mad_icp",
            "--data-path", str(bin_dir.resolve()),
            "--estimate-path", str(mad_out_dir.resolve()),
            "--dataset-config", str(dataset_cfg.resolve()),
            "--num-cores", str(num_cores),
            "--num-keyframes", str(num_keyframes),
            "--noviz",
        ],
        capture_output=True, text=True,
    )
    if res.returncode != 0:
        print(res.stdout); print(res.stderr, file=sys.stderr)
        sys.exit(f"  mad_icp failed (exit {res.returncode})")
    estimate = mad_out_dir / "estimate.txt"
    if not estimate.exists():
        sys.exit(f"  mad_icp produced no estimate.txt under {mad_out_dir}")
    print(f"  mad_icp OK, poses at {estimate}")
    return estimate


def kitti_to_tum(estimate_txt: Path, cleaned_dir: Path):
    """Parse KITTI 12-scalar-row poses and inject real PCD timestamps.

    Each line: r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
    (top 3 rows of the 4×4 homogeneous T_world_lidar)
    """
    poses = np.loadtxt(estimate_txt).reshape(-1, 3, 4)
    pcd_ts = np.array(sorted(int(p.stem) for p in cleaned_dir.glob("*.pcd"))) / 1e9
    n = min(len(poses), len(pcd_ts))
    out = np.zeros((n, 8))
    for i in range(n):
        T = poses[i]
        tx, ty, tz = T[0, 3], T[1, 3], T[2, 3]
        qx, qy, qz, qw = matrix_to_quaternion(T[:3, :3])
        out[i] = [pcd_ts[i], tx, ty, tz, qx, qy, qz, qw]
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("recording_dir", type=Path)
    p.add_argument("--primary-lidar", default="remote_front_left_pointcloud")
    p.add_argument("--output-dir", type=Path, required=True)
    p.add_argument("--voxel-size", type=float, default=0.3)
    p.add_argument("--cleaned-pcd-dir", type=Path, default=None,
                   help="Reuse existing cleaned-PCDs dir (e.g. from KISS-ICP).")
    p.add_argument("--num-cores", type=int, default=4)
    p.add_argument("--num-keyframes", type=int, default=4)
    p.add_argument("--max-range", type=float, default=200.0)
    p.add_argument("--min-range", type=float, default=1.0)
    args = p.parse_args()

    rec = args.recording_dir
    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)

    print("=== MAD-ICP backend ===")
    print(f"  recording: {rec}")
    print(f"  primary:   {args.primary_lidar}")
    print(f"  output:    {out}")

    # 1. clean (reuse from prior KISS-ICP / GenZ-ICP run if available)
    src_pcd_dir = rec / "raw_pointclouds" / args.primary_lidar
    if args.cleaned_pcd_dir is not None and args.cleaned_pcd_dir.exists():
        cleaned_dir = args.cleaned_pcd_dir
        print(f"  reusing cleaned PCDs: {cleaned_dir}")
    else:
        cleaned_dir = out / "cleaned_pcds"
        clean_pcds(src_pcd_dir, cleaned_dir)

    # 2. PCD → KITTI .bin
    bin_dir = out / "mad_raw" / "bins"
    if not bin_dir.exists() or not list(bin_dir.glob("*.bin")):
        pcds_to_kitti_bin(cleaned_dir, bin_dir)
    else:
        print(f"  reusing existing bins: {bin_dir}")

    # 3. Write dataset config
    dataset_cfg = out / "mad_raw" / "dataset.cfg"
    write_dataset_config(dataset_cfg, min_range=args.min_range,
                          max_range=args.max_range, sensor_hz=10.0, deskew=False)

    # 4. Run mad_icp
    mad_out = out / "mad_raw"
    estimate_txt = run_mad(bin_dir, dataset_cfg, mad_out,
                            num_cores=args.num_cores,
                            num_keyframes=args.num_keyframes)

    # 5. Parse KITTI → TUM with real timestamps
    data_lidar = kitti_to_tum(estimate_txt, cleaned_dir)
    write_tum(out / "trajectory_lidar.txt", data_lidar,
              "MAD-ICP T_world_lidar (LiDAR frame)")
    print(f"  trajectory_lidar.txt: {len(data_lidar)} poses")

    # 6. compose to baselink
    frame_name = LIDAR_FRAME.get(args.primary_lidar)
    if not frame_name:
        sys.exit(f"  unknown primary lidar '{args.primary_lidar}'")
    T_baselink_lidar = load_extrinsic(rec / "application.yaml", frame_name)
    if T_baselink_lidar is None:
        sys.exit(f"  could not load {frame_name} extrinsic")
    T_lidar_baselink = invert_transform(T_baselink_lidar)
    data_baselink = compose_to_baselink(data_lidar, T_lidar_baselink)
    write_tum(out / "trajectory_baselink.txt", data_baselink,
              "MAD-ICP T_world_baselink")

    # 7. compose baselink → fake IMU (B2 compat)
    T_baselink_imu = load_extrinsic(rec / "application.yaml", "FRAME_GNSS_IMU")
    if T_baselink_imu is None:
        print("  warning: FRAME_GNSS_IMU not found; identity")
        T_baselink_imu = np.eye(4)
    data_imu = compose_baselink_to_imu(data_baselink, T_baselink_imu)
    write_tum(out / "trajectory.txt", data_imu,
              "MAD-ICP T_world_imu (synthetic, for B2 pipeline compat)")

    # 8. stitch raw map (same as KISS / GenZ flow)
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
