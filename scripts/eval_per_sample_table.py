#!/usr/bin/env python3
"""Build a 3-sample × 6-backend × 3-secondary |Δt| comparison table.

For each (sample, secondary), highlight which backend wins on |Δt|.
"""
import json
from pathlib import Path
import numpy as np
import yaml

SAMPLES = ["ZL11626", "ZL10359", "ZL10966", "ZL10968", "ZL11881", "ZL12332", "ZL12382"]
BACKENDS = [
    ("FAST-LIO", "ghcr_run_v3"),
    ("KISS-ICP", "kiss_icp_run"),
    ("LIO-SAM",  "liosam_run"),
    ("LIO-SAM*", "liosam_run_hybrid"),
    ("GenZ-ICP", "genz_icp_run"),
    ("MAD-ICP",  "mad_icp_run"),
    ("LIVO2",    "fastlivo2_run"),
]
SECONDARIES = [("flash_front", "flash_front_pointcloud"),
               ("flash_rear",  "flash_rear_pointcloud"),
               ("rfr",         "remote_front_right_pointcloud")]


def find_calib(sample_dir, backend_dir):
    base = Path("output/multi_sample") / sample_dir / backend_dir
    # Prefer the canonical B2 info-weighted result. For ZL11626 the older
    # `calibrated_extrinsics.yaml` predates the B2 method (§9 / §11 era);
    # for new samples our orchestrator emits `calibrated_extrinsics.yaml`
    # using B2 info-weighting directly.
    for fname in ("calib_B2_pl_infow.yaml", "calibrated_extrinsics.yaml"):
        p = base / fname
        if p.exists():
            return yaml.safe_load(open(p)).get("calibrated_extrinsics", {})
    return {}


def main():
    print(f"{'Sample':<8} {'Secondary':<11} | "
          + " | ".join(f"{name:>10}" for name, _ in BACKENDS))
    print("-" * 100)

    rows = []
    for sample in SAMPLES:
        sample_dir = Path("output/multi_sample") / sample
        if not sample_dir.exists():
            continue
        for sec_short, sec_full in SECONDARIES:
            line = f"{sample:<8} {sec_short:<11} |"
            vals = []
            for bname, bdir in BACKENDS:
                calib = find_calib(sample, bdir)
                rec = calib.get(sec_full, {})
                dt = rec.get("delta_translation_norm_m")
                vals.append(dt)
                line += f" {(f'{dt:.3f}m' if dt is not None else '   --   '):>10} |"
            valid = [(i, v) for i, v in enumerate(vals) if v is not None]
            best_i = min(valid, key=lambda x: x[1])[0] if valid else None
            print(line)
            rows.append((sample, sec_short, vals, best_i))

    print("\n=== Wins per backend ===")
    wins = {bname: 0 for bname, _ in BACKENDS}
    for _, _, _, bi in rows:
        if bi is not None:
            wins[BACKENDS[bi][0]] += 1
    total = sum(wins.values())
    for bname, _ in BACKENDS:
        print(f"  {bname:<10}: {wins[bname]:>2}/{total} {'★' * wins[bname]}")

    # Also report which (sample, secondary) cells have all backends |dt| > 0.5
    print("\n=== Hard cells (best |dt| > 0.5 m, indicates the recording is itself difficult) ===")
    for sample, sec, vals, bi in rows:
        valid_vals = [v for v in vals if v is not None]
        if valid_vals and min(valid_vals) > 0.5:
            print(f"  {sample}/{sec}: best |dt| = {min(valid_vals):.3f}m  (winner: {BACKENDS[bi][0]})")


if __name__ == "__main__":
    main()
