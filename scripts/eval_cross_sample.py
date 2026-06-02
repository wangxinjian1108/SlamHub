#!/usr/bin/env python3
"""Cross-sample stability analysis: same backend across multiple recordings.

For each (sample, backend) pair, load:
  - trajectory.txt (TUM)
  - calibrated_extrinsics.yaml (B2 output)

Then per-backend report:
  - trajectory length / Z drift std consistency across samples
  - B2 |dt| consistency (does the same secondary's |dt| stay similar?)
  - B2 calibration translation consistency (do we get the same xyz across
    samples for the same secondary? — the *true* extrinsic should be stable)

This is the key multi-sample question: a backend that happens to win on one
60s segment might be lucky; we want backends whose precision and consistency
hold across samples.
"""
import argparse
import json
from pathlib import Path
import numpy as np
import yaml


SAMPLES_DEFAULT = [
    ("ZL11626", Path("output/multi_sample/ZL11626")),
    ("ZL10359", Path("output/multi_sample/ZL10359")),
    ("ZL10966", Path("output/multi_sample/ZL10966")),
    ("ZL10968", Path("output/multi_sample/ZL10968")),
    ("ZL11881", Path("output/multi_sample/ZL11881")),
    ("ZL12332", Path("output/multi_sample/ZL12332")),
    ("ZL12382", Path("output/multi_sample/ZL12382")),
]
BACKENDS = [
    # Try canonical B2 file first; fall back to plain calibrated_extrinsics.
    ("FAST-LIO", "ghcr_run_v3", ["calib_B2_pl_infow.yaml", "calibrated_extrinsics.yaml"]),
    ("KISS-ICP", "kiss_icp_run", ["calibrated_extrinsics.yaml"]),
    ("LIO-SAM",  "liosam_run",   ["calibrated_extrinsics.yaml"]),
    ("LIO-SAM*", "liosam_run_hybrid", ["calibrated_extrinsics.yaml"]),
    ("GenZ-ICP", "genz_icp_run", ["calibrated_extrinsics.yaml"]),
    ("MAD-ICP",  "mad_icp_run",  ["calibrated_extrinsics.yaml"]),
]
SECONDARIES = ["flash_front_pointcloud", "flash_rear_pointcloud", "remote_front_right_pointcloud"]


def load_traj(path: Path):
    if not path.exists():
        return None
    d = np.loadtxt(path, comments="#")
    if d.ndim == 1 and len(d) >= 8:
        d = d.reshape(1, -1)
    if d.size == 0:
        return None
    return d


def trajectory_summary(traj):
    if traj is None or len(traj) < 2:
        return None
    xyz = traj[:, 1:4]
    L = float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum())
    return {
        "n_poses": int(len(traj)),
        "length_m": L,
        "z_std": float(xyz[:, 2].std()),
        "z_min": float(xyz[:, 2].min()),
        "z_max": float(xyz[:, 2].max()),
    }


def load_calib(paths):
    for path in paths:
        if path.exists():
            d = yaml.safe_load(open(path))
            return d.get("calibrated_extrinsics", d)
    return None


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--out", type=Path,
                   default=Path("output/multi_sample/cross_sample_summary.json"))
    args = p.parse_args()

    samples = []
    for sname, sdir in SAMPLES_DEFAULT:
        if not sdir.exists():
            print(f"  skip {sname} (no dir at {sdir})")
            continue
        samples.append((sname, sdir))
    if not samples:
        print("No samples found.")
        return

    out = {"samples": [s[0] for s in samples], "backends": {}}

    for backend_name, backend_dir, calib_yamls in BACKENDS:
        out["backends"][backend_name] = {
            "trajectory": {},
            "calibration": {sec: {} for sec in SECONDARIES},
        }

        # Per-sample trajectory summary
        for sname, sdir in samples:
            tj_path = sdir / backend_dir / "trajectory.txt"
            summary = trajectory_summary(load_traj(tj_path))
            if summary is not None:
                out["backends"][backend_name]["trajectory"][sname] = summary

        # Per-sample, per-secondary B2 calibration (try multiple filenames)
        for sname, sdir in samples:
            paths = [sdir / backend_dir / fn for fn in calib_yamls]
            calib = load_calib(paths)
            if not calib:
                continue
            for sec in SECONDARIES:
                if sec not in calib:
                    continue
                rec = calib[sec]
                out["backends"][backend_name]["calibration"][sec][sname] = {
                    "xyz": list(rec.get("translation_xyz_m", [None]*3)),
                    "rpy": list(rec.get("euler_rpy_rad", [None]*3)),
                    "std":  list(rec.get("translation_std_m", [None]*3)),
                    "n_eff": rec.get("n_effective_weighted"),
                    "dt_norm": rec.get("delta_translation_norm_m"),
                }

    # Aggregate stability stats: how much does the same calibration estimate
    # vary across samples (using the same backend)?
    print(f"{'Backend':<10} {'Secondary':<14} {'samples':>7}  {'xyz_std (m)':>30}  {'|dt| range':>20}")
    print("-" * 90)
    summary_rows = []
    for bname, bdata in out["backends"].items():
        for sec in SECONDARIES:
            sec_results = bdata["calibration"][sec]
            xyz_arr = np.array([v["xyz"] for v in sec_results.values()
                                if all(x is not None for x in v["xyz"])])
            if len(xyz_arr) < 2:
                continue
            xyz_std = xyz_arr.std(axis=0)
            dt_norms = [v["dt_norm"] for v in sec_results.values()
                        if v.get("dt_norm") is not None]
            row = {
                "backend": bname, "secondary": sec, "n": len(xyz_arr),
                "xyz_std_xyz": xyz_std.tolist(),
                "xyz_std_norm": float(np.linalg.norm(xyz_std)),
                "dt_norm_min": float(min(dt_norms)) if dt_norms else None,
                "dt_norm_max": float(max(dt_norms)) if dt_norms else None,
            }
            summary_rows.append(row)
            short_sec = {"flash_front_pointcloud": "flash_front",
                         "flash_rear_pointcloud":  "flash_rear",
                         "remote_front_right_pointcloud": "rfr"}[sec]
            xyz_str = (f"x{xyz_std[0]:.3f} y{xyz_std[1]:.3f} z{xyz_std[2]:.3f}"
                       if len(xyz_std) == 3 else "n/a")
            dt_range = (f"{row['dt_norm_min']:.3f}–{row['dt_norm_max']:.3f}"
                        if row['dt_norm_min'] is not None else "n/a")
            print(f"{bname:<10} {short_sec:<14} {row['n']:>7}  {xyz_str:>30}  {dt_range:>20}")

    out["stability"] = summary_rows
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(out, f, indent=2)
    print(f"\nWrote {args.out}")


if __name__ == "__main__":
    main()
