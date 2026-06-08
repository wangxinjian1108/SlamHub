#!/usr/bin/env python3
"""3-way comparison: cross-LiDAR calibration using FAST-LIO vs KISS-ICP vs LIO-SAM as the primary map."""
import json
from pathlib import Path
import yaml


def load_calib(path, key="calibrated_extrinsics"):
    cfg = yaml.safe_load(open(path))
    return cfg.get(key, cfg)


def main():
    inputs = {
        "FAST-LIO":  Path("output/ghcr_run_v3/calib_B2_pl_infow.yaml"),
        "KISS-ICP":  Path("output/kiss_icp_run/calibrated_extrinsics.yaml"),
        "LIO-SAM":   Path("output/liosam_run/calibrated_extrinsics.yaml"),
        "LIO-SAM*":  Path("output/liosam_run_hybrid/calibrated_extrinsics.yaml"),
        "GenZ-ICP":  Path("output/genz_icp_run/calibrated_extrinsics.yaml"),
        "MAD-ICP":   Path("output/mad_icp_run/calibrated_extrinsics.yaml"),
        "LIVO2":     Path("output/fastlivo2_run/calibrated_extrinsics.yaml"),
    }
    calibs = {name: load_calib(p) for name, p in inputs.items()}

    secondaries = ["flash_front_pointcloud", "flash_rear_pointcloud", "remote_front_right_pointcloud"]
    short = {"flash_front_pointcloud": "flash_front",
             "flash_rear_pointcloud":  "flash_rear",
             "remote_front_right_pointcloud": "rfr"}

    rows = []
    for sec in secondaries:
        for name in inputs:
            r = calibs[name].get(sec, {})
            std = r.get("translation_std_m", [None]*3)
            d = r.get("delta_translation_m")
            n_eff = r.get("n_effective_weighted")
            dnorm = r.get("delta_translation_norm_m")
            rows.append({
                "secondary": short[sec], "primary": name,
                "dx_std": std[0] if std and std[0] is not None else None,
                "dy_std": std[1] if std and std[1] is not None else None,
                "dz_std": std[2] if std and std[2] is not None else None,
                "n_eff":  n_eff,
                "dt_norm": dnorm,
            })

    print(f"{'secondary':<11} {'primary':<10} {'dx_std':>7} {'dy_std':>7} {'dz_std':>7} {'n_eff':>6} {'|dt|':>6}")
    print("-" * 62)
    for r in rows:
        d = lambda v, fmt: f"{v:{fmt}}" if v is not None else "  --  "
        print(f"{r['secondary']:<11} {r['primary']:<10} "
              f"{d(r['dx_std'], '7.3f')} {d(r['dy_std'], '7.3f')} {d(r['dz_std'], '7.3f')} "
              f"{d(r['n_eff'], '6.1f')} {d(r['dt_norm'], '6.3f')}")

    out = Path("output/three_way_compare/calibration_three_way.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w") as f:
        json.dump(rows, f, indent=2)
    print(f"\nWrote {out}")


if __name__ == "__main__":
    main()
