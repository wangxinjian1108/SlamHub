#!/usr/bin/env python3
"""Per-(sample, secondary) side-by-side calibration values across all backends.

For each (sample, secondary) cell, show:
  - Each backend's calibrated translation_xyz_m and euler_rpy_rad
  - Median across backends (consensus value when GT is unknown)
  - Each backend's delta vs the median (signed, per axis)
  - Std across backends (= eval_internal_quality.py's xyz_std)
  - Initial guess from application.yaml (for reference, not GT)

This is the "where do the methods disagree" view that |dt| summarizes away.
"""
from pathlib import Path
import argparse
import yaml
import numpy as np

SAMPLES = ["ZL11626", "ZL10359", "ZL10966", "ZL10968", "ZL11881", "ZL12332", "ZL12382"]
BACKENDS = [
    ("FAST-LIO", "ghcr_run_v3", ["calib_B2_pl_infow.yaml", "calibrated_extrinsics.yaml"]),
    ("KISS-ICP", "kiss_icp_run", ["calibrated_extrinsics.yaml"]),
    ("LIO-SAM",  "liosam_run",   ["calibrated_extrinsics.yaml"]),
    ("LIO-SAM*", "liosam_run_hybrid", ["calibrated_extrinsics.yaml"]),
    ("GenZ-ICP", "genz_icp_run", ["calibrated_extrinsics.yaml"]),
    ("MAD-ICP",  "mad_icp_run",  ["calibrated_extrinsics.yaml"]),
    ("LIVO2",    "fastlivo2_run", ["calibrated_extrinsics.yaml"]),
]
SECONDARIES = [("flash_front", "flash_front_pointcloud"),
               ("flash_rear",  "flash_rear_pointcloud"),
               ("rfr",         "remote_front_right_pointcloud")]

DATA_BASE = Path("/root/code/LargeCalibService/node_data/fixtures/lio")
SAMPLE_TO_DIR = {
    "ZL11626": "ZL11626_40482_zelos_sample_2025-07-02_14-29-00_000000000_8993544",
    "ZL10359": "ZL10359_ALL_20260527123524-20260527123549",
    "ZL10966": "ZL10966_ALL_20260527123659-20260527123719",
    "ZL10968": "ZL10968_ALL_20250625151932-20250625152000",
    "ZL11881": "ZL11881_ALL_20250717143903-20250717143933",
    "ZL12332": "ZL12332_ALL_20260527133807-20260527133907",
    "ZL12382": "ZL12382_ALL_20260527134536-20260527134636",
}
LIDAR_FRAME = {
    "flash_front_pointcloud": "FRAME_LIDAR_FLASH_FRONT",
    "flash_rear_pointcloud":  "FRAME_LIDAR_FLASH_REAR",
    "remote_front_right_pointcloud": "FRAME_LIDAR_REMOTE_FRONT_RIGHT",
}
ROOT = Path("output/multi_sample")


def find_calib(sample, backend_dir, fns):
    for fn in fns:
        p = ROOT / sample / backend_dir / fn
        if p.exists():
            return yaml.safe_load(open(p)).get("calibrated_extrinsics", {})
    return None


def initial_guess(sample, sec_full):
    yaml_path = DATA_BASE / SAMPLE_TO_DIR[sample] / "application.yaml"
    cfg = yaml.safe_load(open(yaml_path))
    target = LIDAR_FRAME[sec_full]
    for cal in cfg["vehicle"]["calibration"]["sensor_calibration"]:
        if cal.get("source") == target:
            t = cal["transformation"]
            return t  # [x, y, z, roll, pitch, yaw]
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sample", help="filter to one sample")
    ap.add_argument("--secondary", help="filter to one secondary (short name)")
    args = ap.parse_args()

    samples_to_show = [s for s in SAMPLES if args.sample is None or s == args.sample]
    secs_to_show = [(short, full) for short, full in SECONDARIES
                    if args.secondary is None or short == args.secondary]

    for sample in samples_to_show:
        if not (ROOT / sample).exists():
            continue
        for sec_short, sec_full in secs_to_show:
            init = initial_guess(sample, sec_full)
            if init is None:
                continue
            init_xyz = np.array(init[:3])
            init_rpy = np.array(init[3:])

            # Gather per-backend
            rows = []
            for bname, bdir, files in BACKENDS:
                calib = find_calib(sample, bdir, files)
                if not calib or sec_full not in calib:
                    continue
                rec = calib[sec_full]
                xyz = rec.get("translation_xyz_m")
                rpy = rec.get("euler_rpy_rad")
                if xyz is None or rpy is None:
                    continue
                rows.append((bname, np.array(xyz), np.array(rpy)))

            if len(rows) < 2:
                continue

            xyzs = np.stack([r[1] for r in rows])
            rpys = np.stack([r[2] for r in rows])
            median_xyz = np.median(xyzs, axis=0)
            std_xyz = xyzs.std(axis=0)
            median_rpy = np.median(rpys, axis=0)
            std_rpy = rpys.std(axis=0)

            print(f"\n========== {sample} / {sec_short} ==========")
            print(f"  init (factory)  : "
                  f"x={init_xyz[0]:+.4f}m y={init_xyz[1]:+.4f}m z={init_xyz[2]:+.4f}m  "
                  f"r={np.degrees(init_rpy[0]):+.3f}° p={np.degrees(init_rpy[1]):+.3f}° y={np.degrees(init_rpy[2]):+.3f}°")
            print(f"  consensus median: "
                  f"x={median_xyz[0]:+.4f}m y={median_xyz[1]:+.4f}m z={median_xyz[2]:+.4f}m  "
                  f"r={np.degrees(median_rpy[0]):+.3f}° p={np.degrees(median_rpy[1]):+.3f}° y={np.degrees(median_rpy[2]):+.3f}°")
            print(f"  cross-backend std: "
                  f"x={std_xyz[0]:.4f}m y={std_xyz[1]:.4f}m z={std_xyz[2]:.4f}m  "
                  f"r={np.degrees(std_rpy[0]):.3f}° p={np.degrees(std_rpy[1]):.3f}° y={np.degrees(std_rpy[2]):.3f}°")
            print()
            print(f"  {'Backend':<10} {'x (m)':>9} {'y (m)':>9} {'z (m)':>9} | "
                  f"{'Δx':>7} {'Δy':>7} {'Δz':>7} | "
                  f"{'r (°)':>8} {'p (°)':>8} {'y (°)':>8}")
            print(f"  {'-'*10} {'-'*9} {'-'*9} {'-'*9} + "
                  f"{'-'*7} {'-'*7} {'-'*7} + {'-'*8} {'-'*8} {'-'*8}")
            for bname, xyz, rpy in rows:
                d = xyz - median_xyz
                rpy_deg = np.degrees(rpy)
                print(f"  {bname:<10} "
                      f"{xyz[0]:+9.4f} {xyz[1]:+9.4f} {xyz[2]:+9.4f} | "
                      f"{d[0]:+7.3f} {d[1]:+7.3f} {d[2]:+7.3f} | "
                      f"{rpy_deg[0]:+8.3f} {rpy_deg[1]:+8.3f} {rpy_deg[2]:+8.3f}")


if __name__ == "__main__":
    main()
