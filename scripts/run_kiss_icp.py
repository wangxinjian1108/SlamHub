#!/usr/bin/env python3
"""Run KISS-ICP backend end-to-end on a recording directory.

Pipeline:
  1. Pre-clean: drop NaN points from primary LiDAR PCDs
     (KISS-ICP silently exits on NaN — found the hard way)
  2. Run kiss_icp_pipeline on the cleaned PCD dir
  3. Inject real timestamps from PCD filenames into the TUM trajectory
  4. Compose to baselink frame:  T_world_baselink = T_world_lidar @ T_lidar_baselink
  5. Compose to "fake IMU" frame: T_world_imu = T_world_baselink @ T_baselink_imu
     (so the rest of the SlamHub pipeline — which assumes FAST-LIO-style
      T_world_imu input — works unchanged)
  6. Stitch a global map from the per-frame poses + cleaned PCDs,
     voxel-downsample to 0.3 m

Outputs in --output-dir:
  trajectory.txt          T_world_imu, TUM format
  scans.pcd               full accumulated map
  scans_voxel0.3.pcd      downsampled (for ICP target)
  kiss_raw/               KISS-ICP raw output (for debugging)
  cleaned_pcds/           NaN-free intermediate (large; can rm after)

Required dependencies: kiss-icp, open3d, numpy, pyyaml
"""
import argparse
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).parent))
from convert_to_rosbag_velodyne import parse_pcd_binary
from common.io import write_pcd
from common.transform import (
    euler_to_matrix, quaternion_to_matrix, matrix_to_quaternion,
    make_homogeneous, invert_transform,
)


LIDAR_FRAME = {
    "remote_front_left_pointcloud": "FRAME_LIDAR_REMOTE_FRONT_LEFT",
    "remote_front_right_pointcloud": "FRAME_LIDAR_REMOTE_FRONT_RIGHT",
    "flash_front_pointcloud": "FRAME_LIDAR_FLASH_FRONT",
    "flash_rear_pointcloud": "FRAME_LIDAR_FLASH_REAR",
}


def load_extrinsic(yaml_path, frame_name):
    cfg = yaml.safe_load(open(yaml_path))
    for cal in cfg["vehicle"]["calibration"]["sensor_calibration"]:
        if cal["source"] == frame_name:
            t = cal["transformation"]
            R = euler_to_matrix(t[3], t[4], t[5])
            return make_homogeneous(R, np.array(t[:3]))
    return None


def clean_pcds(src_dir, dst_dir):
    """Strip NaN points; write 3-field (xyz) binary PCDs."""
    dst_dir.mkdir(parents=True, exist_ok=True)
    files = sorted(src_dir.glob("*.pcd"))
    print(f"  cleaning {len(files)} PCDs...")
    for i, f in enumerate(files):
        d = parse_pcd_binary(f)
        valid = d[~np.isnan(d["x"])]
        n = len(valid)
        if n == 0:
            continue
        pts = np.zeros((n, 3), dtype=np.float32)
        pts[:, 0] = valid["x"]
        pts[:, 1] = valid["y"]
        pts[:, 2] = valid["z"]
        write_pcd(dst_dir / f.name, pts)
        if (i + 1) % 200 == 0:
            print(f"    {i+1}/{len(files)}")
    print(f"  done — {len(files)} PCDs in {dst_dir}")


def run_kiss(cleaned_dir, raw_out_dir):
    """Invoke kiss_icp_pipeline, return path to TUM poses file."""
    raw_out_dir.mkdir(parents=True, exist_ok=True)
    # kiss_icp_pipeline writes results into <cwd>/results/<timestamp>/
    cwd = raw_out_dir
    print(f"  running kiss_icp_pipeline on {cleaned_dir}...")
    res = subprocess.run(
        ["kiss_icp_pipeline", str(cleaned_dir)],
        cwd=str(cwd),
        capture_output=True, text=True,
    )
    if res.returncode != 0:
        print(res.stdout)
        print(res.stderr, file=sys.stderr)
        sys.exit(f"  kiss_icp_pipeline failed (exit {res.returncode})")
    # Find newest results/<ts>/<basename>_poses_tum.txt
    results = sorted((raw_out_dir / "results").glob("*"))
    if not results:
        sys.exit(f"  no results/ produced under {raw_out_dir}")
    latest = results[-1]
    tum_files = list(latest.glob("*_poses_tum.txt"))
    if not tum_files:
        sys.exit(f"  no _poses_tum.txt in {latest}")
    print(f"  kiss-icp OK, poses at {tum_files[0]}")
    return tum_files[0]


def inject_timestamps(kiss_tum, cleaned_dir):
    """Replace KISS-ICP's frame-index timestamps with real ones from PCD filenames."""
    data = np.loadtxt(kiss_tum)
    ts_ns = sorted(int(f.stem) for f in cleaned_dir.glob("*.pcd"))
    ts_s = np.array(ts_ns) / 1e9
    n = min(len(data), len(ts_s))
    data = data[:n].copy()
    data[:, 0] = ts_s[:n]
    return data


def compose_to_baselink(data_lidar, T_lidar_baselink):
    """Convert each pose from LiDAR frame to baselink frame:
       T_world_baselink = T_world_lidar @ T_lidar_baselink"""
    out = np.zeros_like(data_lidar)
    for i, row in enumerate(data_lidar):
        ts, tx, ty, tz, qx, qy, qz, qw = row
        R = quaternion_to_matrix(qx, qy, qz, qw)
        T_wl = make_homogeneous(R, np.array([tx, ty, tz]))
        T_wb = T_wl @ T_lidar_baselink
        qx2, qy2, qz2, qw2 = matrix_to_quaternion(T_wb[:3, :3])
        out[i] = [ts, T_wb[0, 3], T_wb[1, 3], T_wb[2, 3], qx2, qy2, qz2, qw2]
    return out


def compose_baselink_to_imu(data_baselink, T_baselink_imu):
    """T_world_imu = T_world_baselink @ T_baselink_imu — fake IMU trajectory
    so the existing B2 pipeline (which assumes FAST-LIO T_world_imu) works."""
    out = np.zeros_like(data_baselink)
    for i, row in enumerate(data_baselink):
        ts, tx, ty, tz, qx, qy, qz, qw = row
        R = quaternion_to_matrix(qx, qy, qz, qw)
        T_wb = make_homogeneous(R, np.array([tx, ty, tz]))
        T_wi = T_wb @ T_baselink_imu
        qx2, qy2, qz2, qw2 = matrix_to_quaternion(T_wi[:3, :3])
        out[i] = [ts, T_wi[0, 3], T_wi[1, 3], T_wi[2, 3], qx2, qy2, qz2, qw2]
    return out


def write_tum(path, data, header_comment):
    with open(path, "w") as f:
        f.write(f"# {header_comment}\n")
        for row in data:
            f.write(" ".join(f"{v:.9f}" for v in row) + "\n")


def stitch_map(cleaned_dir, traj_lidar, out_full, out_voxel, voxel=0.3):
    """Build global map from per-frame poses + cleaned PCDs (LiDAR frame)."""
    import open3d as o3d

    files = sorted(cleaned_dir.glob("*.pcd"))
    n = min(len(files), len(traj_lidar))
    all_pts = []
    for i in range(n):
        pts = parse_pcd_binary(files[i])
        xyz = np.column_stack([pts["x"], pts["y"], pts["z"]]).astype(np.float32)
        _, tx, ty, tz, qx, qy, qz, qw = traj_lidar[i]
        R = quaternion_to_matrix(qx, qy, qz, qw)
        T = make_homogeneous(R, np.array([tx, ty, tz]))
        pts_h = np.hstack([xyz, np.ones((len(xyz), 1))])
        all_pts.append((T @ pts_h.T).T[:, :3])
        if (i + 1) % 100 == 0:
            print(f"    stitched {i+1}/{n}")
    all_pts = np.vstack(all_pts).astype(np.float32)
    print(f"  total {len(all_pts):,} points; writing full map...")
    write_pcd(out_full, all_pts)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(all_pts.astype(np.float64))
    ds = pcd.voxel_down_sample(voxel)
    print(f"  voxel {voxel} downsample → {len(ds.points):,} points; writing...")
    o3d.io.write_point_cloud(str(out_voxel), ds)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("recording_dir", type=Path)
    p.add_argument("--primary-lidar", default="remote_front_left_pointcloud")
    p.add_argument("--output-dir", type=Path, required=True)
    p.add_argument("--voxel-size", type=float, default=0.3)
    p.add_argument("--skip-clean", action="store_true",
                   help="Reuse existing cleaned_pcds/ if present")
    args = p.parse_args()

    rec = args.recording_dir
    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)

    print("=== KISS-ICP backend ===")
    print(f"  recording: {rec}")
    print(f"  primary:   {args.primary_lidar}")
    print(f"  output:    {out}")

    # 1. clean
    src_pcd_dir = rec / "raw_pointclouds" / args.primary_lidar
    cleaned_dir = out / "cleaned_pcds"
    if args.skip_clean and cleaned_dir.exists() and any(cleaned_dir.glob("*.pcd")):
        print(f"  skip clean (reusing {cleaned_dir})")
    else:
        clean_pcds(src_pcd_dir, cleaned_dir)

    # 2. KISS-ICP
    raw_out = out / "kiss_raw"
    kiss_tum = run_kiss(cleaned_dir, raw_out)

    # 3. inject timestamps
    data_lidar = inject_timestamps(kiss_tum, cleaned_dir)
    write_tum(out / "trajectory_lidar.txt", data_lidar,
              "KISS-ICP T_world_lidar (LiDAR frame)")
    print(f"  trajectory_lidar.txt: {len(data_lidar)} poses")

    # 4. compose to baselink
    frame_name = LIDAR_FRAME.get(args.primary_lidar)
    if not frame_name:
        sys.exit(f"  unknown primary lidar '{args.primary_lidar}' (no LIDAR_FRAME entry)")
    T_baselink_lidar = load_extrinsic(rec / "application.yaml", frame_name)
    if T_baselink_lidar is None:
        sys.exit(f"  could not load {frame_name} extrinsic from application.yaml")
    T_lidar_baselink = invert_transform(T_baselink_lidar)
    data_baselink = compose_to_baselink(data_lidar, T_lidar_baselink)
    write_tum(out / "trajectory_baselink.txt", data_baselink,
              "KISS-ICP T_world_baselink")

    # 5. compose baselink → fake IMU (for B2 compat)
    T_baselink_imu = load_extrinsic(rec / "application.yaml", "FRAME_GNSS_IMU")
    if T_baselink_imu is None:
        print("  warning: FRAME_GNSS_IMU not found; trajectory.txt == trajectory_baselink.txt")
        T_baselink_imu = np.eye(4)
    data_imu = compose_baselink_to_imu(data_baselink, T_baselink_imu)
    write_tum(out / "trajectory.txt", data_imu,
              "KISS-ICP T_world_imu (synthetic, for B2 pipeline compat)")

    # 6. stitch map
    stitch_map(cleaned_dir, data_lidar,
               out / "scans.pcd",
               out / f"scans_voxel{args.voxel_size}.pcd",
               voxel=args.voxel_size)

    print("=== Done ===")
    print(f"  trajectory.txt:        {out / 'trajectory.txt'}")
    print(f"  trajectory_baselink.txt: {out / 'trajectory_baselink.txt'}")
    print(f"  scans.pcd:             {out / 'scans.pcd'}")
    print(f"  scans_voxel{args.voxel_size}.pcd: {out / f'scans_voxel{args.voxel_size}.pcd'}")


if __name__ == "__main__":
    main()
