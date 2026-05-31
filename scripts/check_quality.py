#!/usr/bin/env python3
"""Quality alarms for SlamHub pipeline outputs.

Reads the artifacts produced by run_all.sh / extract_extrinsic / compare and
emits PASS / WARN / FAIL with reasons. Returns nonzero exit code on FAIL so
CI / batch runners can flag bad runs.

Checked artifacts (any of these may be missing):
  output/<run>/scans.pcd
  output/<run>/trajectory.txt
  output/<run>/compare_traj_summary.yaml
  output/<run>/registration_pl2/<lidar>/summary.yaml
  output/<run>/calibrated_extrinsics.yaml

Thresholds are intentionally loose so they only fire on obvious problems;
production sites should tighten via the YAML thresholds file or CLI flags.
"""
import argparse
import sys
from pathlib import Path
import numpy as np
import yaml


# (key, fail level, warn-level) — chosen from §2/§3 baselines
DEFAULT_THRESHOLDS = {
    # primary SLAM trajectory vs LIDAR_TO_MAP
    "ate_rms_m": dict(warn=0.5, fail=1.5),
    "rpe_10s_trans_rms_m": dict(warn=0.6, fail=1.5),
    "rot_max_deg": dict(warn=3.0, fail=10.0),
    "z_drift_m": dict(warn=2.0, fail=8.0),  # from first-frame ATE dz_bias

    # cross-LiDAR registration per secondary
    "mean_fitness": dict(warn=0.5, fail=0.3),       # higher is better
    "mean_rmse_m": dict(warn=0.5, fail=1.5),
    "delta_t_norm_m": dict(warn=0.5, fail=1.5),     # |Δt| vs factory
    "calib_translation_std_m": dict(warn=1.0, fail=3.0),
}


class Result:
    def __init__(self):
        self.issues = []          # list of (level, msg)
        self.metrics_seen = []

    def add(self, level, msg):
        self.issues.append((level, msg))

    def status(self):
        if any(lvl == "FAIL" for lvl, _ in self.issues):
            return "FAIL"
        if any(lvl == "WARN" for lvl, _ in self.issues):
            return "WARN"
        return "PASS"

    def print(self):
        for level, msg in self.issues:
            tag = {"FAIL": "❌", "WARN": "⚠️ ", "INFO": "ℹ "}.get(level, " ")
            print(f"{tag} {level:4s}  {msg}")
        print()
        print(f"OVERALL: {self.status()}  "
              f"({sum(1 for l,_ in self.issues if l=='FAIL')} fail / "
              f"{sum(1 for l,_ in self.issues if l=='WARN')} warn / "
              f"{len(self.metrics_seen)} checks)")


def check_metric(res, key, value, thresholds, higher_better=False, label=None):
    res.metrics_seen.append(key)
    th = thresholds.get(key)
    if th is None:
        return
    warn, fail = th["warn"], th["fail"]
    lbl = label or key
    if higher_better:
        # lower is worse — fail < warn < good
        if value < fail:
            res.add("FAIL", f"{lbl} = {value:.4f} < fail thresh {fail}")
        elif value < warn:
            res.add("WARN", f"{lbl} = {value:.4f} < warn thresh {warn}")
        else:
            res.add("INFO", f"{lbl} = {value:.4f} (ok ≥ {warn})")
    else:
        if value > fail:
            res.add("FAIL", f"{lbl} = {value:.4f} > fail thresh {fail}")
        elif value > warn:
            res.add("WARN", f"{lbl} = {value:.4f} > warn thresh {warn}")
        else:
            res.add("INFO", f"{lbl} = {value:.4f} (ok ≤ {warn})")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--run-dir", type=Path, required=True,
                   help="Run output directory (e.g. output/ghcr_run_v4)")
    p.add_argument("--registration-dir", type=Path, default=None,
                   help="Override registration subdir (default: run_dir/registration_pl2)")
    p.add_argument("--thresholds", type=Path, default=None,
                   help="Override default thresholds with a YAML file")
    args = p.parse_args()

    th = dict(DEFAULT_THRESHOLDS)
    if args.thresholds and args.thresholds.exists():
        override = yaml.safe_load(open(args.thresholds))
        for k, v in override.items():
            th[k] = v

    res = Result()
    print(f"=== Quality check on {args.run_dir} ===\n")

    # 1. Trajectory vs LIDAR_TO_MAP summary
    traj_summary_p = args.run_dir / "compare_traj_summary.yaml"
    if traj_summary_p.exists():
        summary = yaml.safe_load(open(traj_summary_p))
        v = summary.get("nearest_global", {})
        check_metric(res, "ate_rms_m", v.get("ate_rms_m", 0.0), th,
                     label="trajectory ATE RMS")
        check_metric(res, "rot_max_deg",
                     v.get("rotation_error_deg", {}).get("max", 0.0), th,
                     label="trajectory rot max")
        first = summary.get("nearest_firstframe", {})
        if first:
            dz = abs(first.get("axis_bias_m", {}).get("dz", 0.0))
            check_metric(res, "z_drift_m", dz, th, label="Z drift (first-frame)")
        rpe = summary.get("rpe", {}).get("window_10s", {})
        if "trans_rms" in rpe:
            check_metric(res, "rpe_10s_trans_rms_m", rpe["trans_rms"], th,
                         label="RPE 10s translation RMS")
    else:
        res.add("WARN", f"missing {traj_summary_p}")

    # 2. Cross-LiDAR registration per-secondary
    reg_dir = args.registration_dir or (args.run_dir / "registration_pl2")
    if reg_dir.exists():
        for sub in sorted(reg_dir.iterdir()):
            if not sub.is_dir():
                continue
            sm = sub / "summary.yaml"
            if not sm.exists():
                res.add("WARN", f"missing {sm}")
                continue
            d = yaml.safe_load(open(sm))
            label_prefix = f"reg/{sub.name}"
            check_metric(res, "mean_fitness", d.get("mean_fitness", 0.0), th,
                         higher_better=True, label=f"{label_prefix} fitness")
            check_metric(res, "mean_rmse_m", d.get("mean_rmse", 0.0), th,
                         label=f"{label_prefix} rmse")
    else:
        res.add("WARN", f"missing {reg_dir}")

    # 3. Calibrated extrinsics: per-LiDAR |Δt| vs factory + std
    calib_p = args.run_dir / "calibrated_extrinsics.yaml"
    if calib_p.exists():
        calib = yaml.safe_load(open(calib_p))
        for name, info in calib.get("calibrated_extrinsics", {}).items():
            d_norm = info.get("delta_translation_norm_m")
            if d_norm is not None:
                check_metric(res, "delta_t_norm_m", d_norm, th,
                             label=f"calib/{name} |Δt vs factory|")
            std = info.get("translation_std_m")
            if std:
                std_norm = float(np.linalg.norm(std))
                check_metric(res, "calib_translation_std_m", std_norm, th,
                             label=f"calib/{name} ‖σ_t‖")
    else:
        res.add("WARN", f"missing {calib_p}")

    res.print()
    sys.exit(2 if res.status() == "FAIL" else 0)


if __name__ == "__main__":
    main()
