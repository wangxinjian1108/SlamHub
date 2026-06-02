#!/usr/bin/env python3
"""GT-free quality metrics for SLAM + cross-LiDAR calibration.

Three families of signal we can extract WITHOUT trusting any reference:

1) Per-frame ICP quality (already saved by 04_register_secondary.py)
   - fitness   ∈ [0,1]: fraction of source points with a near-neighbor
   - inlier_rmse: residual distance of the inliers
   - 6×6 information matrix: diagonal entries are the per-axis Hessian
     (precision) of the local pose estimate; small diag entry = weak axis

2) Aggregated B2 calibration variance (from calibrated_extrinsics.yaml)
   - translation_std_m (per-axis std of 600 frame estimates)
   - n_effective_weighted (effective sample size after info-weighting)

3) Cross-backend consistency (no GT needed)
   - For each (sample, secondary), look at the spread of `translation_xyz_m`
     across all backends. Tight spread = backends agree → likely correct.
     Wide spread = backends disagree → at least one (or all) is wrong.
   - This is the strongest GT-free signal: if 5 different SLAM algorithms
     converge on the same answer, that answer is probably right; if they
     spread by 2 m, none of them is to be trusted.

Usage:
  python eval_internal_quality.py
"""
from pathlib import Path
import json
import numpy as np
import yaml

SAMPLES = ["ZL11626", "ZL10359", "ZL10966", "ZL10968", "ZL11881", "ZL12332", "ZL12382"]
BACKENDS = [
    ("FAST-LIO", "ghcr_run_v3", ["calib_B2_pl_infow.yaml", "calibrated_extrinsics.yaml"]),
    ("KISS-ICP", "kiss_icp_run", ["calibrated_extrinsics.yaml"]),
    ("LIO-SAM",  "liosam_run",   ["calibrated_extrinsics.yaml"]),
    ("LIO-SAM*", "liosam_run_hybrid", ["calibrated_extrinsics.yaml"]),
    ("GenZ-ICP", "genz_icp_run", ["calibrated_extrinsics.yaml"]),
    ("MAD-ICP",  "mad_icp_run",  ["calibrated_extrinsics.yaml"]),
]
SECONDARIES = [("flash_front", "flash_front_pointcloud"),
               ("flash_rear",  "flash_rear_pointcloud"),
               ("rfr",         "remote_front_right_pointcloud")]
ROOT = Path("output/multi_sample")


def load_calib(sample, backend_dir, calib_files):
    base = ROOT / sample / backend_dir
    for fn in calib_files:
        p = base / fn
        if p.exists():
            return yaml.safe_load(open(p)).get("calibrated_extrinsics", {})
    return None


def per_frame_quality(sample, backend_dir, sec_full):
    """Average ICP fitness/rmse + min eigenvalue of mean info matrix."""
    qpath = ROOT / sample / backend_dir / "registration" / sec_full / "frame_quality.csv"
    ipath = ROOT / sample / backend_dir / "registration" / sec_full / "frame_information.csv"
    if not qpath.exists():
        return None
    q = np.loadtxt(qpath, delimiter=",", skiprows=1)
    if q.size == 0:
        return None
    if q.ndim == 1:
        q = q.reshape(1, -1)
    out = {
        "n_frames": int(len(q)),
        "fitness_mean": float(q[:, 1].mean()),
        "fitness_median": float(np.median(q[:, 1])),
        "rmse_mean": float(q[:, 2].mean()),
        "inliers_mean": float(q[:, 3].mean()),
    }
    if ipath.exists():
        info = np.loadtxt(ipath, comments="#")
        if info.ndim == 1:
            info = info.reshape(1, -1)
        if info.size:
            mats = info[:, 1:].reshape(-1, 6, 6)
            mean_info = mats.mean(0)
            try:
                eigs = np.linalg.eigvalsh(mean_info)
                out["info_min_eig"] = float(eigs.min())  # weakest axis precision
                out["info_max_eig"] = float(eigs.max())
                out["info_cond"] = float(eigs.max() / max(eigs.min(), 1e-9))
            except np.linalg.LinAlgError:
                pass
    return out


def cross_backend_spread():
    """For each (sample, secondary), compute std of translation_xyz_m across
    backends. Tight spread = backends agree (likely correct)."""
    spreads = {}
    for sample in SAMPLES:
        if not (ROOT / sample).exists():
            continue
        spreads[sample] = {}
        for sec_short, sec_full in SECONDARIES:
            xyzs = []
            for bname, bdir, files in BACKENDS:
                calib = load_calib(sample, bdir, files)
                if calib and sec_full in calib:
                    t = calib[sec_full].get("translation_xyz_m")
                    if t and all(x is not None for x in t):
                        xyzs.append(t)
            if len(xyzs) >= 2:
                arr = np.array(xyzs)
                spreads[sample][sec_short] = {
                    "n_backends": len(arr),
                    "xyz_mean": arr.mean(0).tolist(),
                    "xyz_std":  arr.std(0).tolist(),
                    "xyz_std_norm": float(np.linalg.norm(arr.std(0))),
                    "max_pairwise_dist": float(np.max([np.linalg.norm(a-b)
                        for i, a in enumerate(arr) for b in arr[i+1:]])),
                }
    return spreads


def main():
    out = {"per_frame_quality": {}, "b2_variance": {}, "cross_backend_spread": {}}

    print("=== Per-frame ICP quality (cross-LiDAR registration) ===")
    print(f"{'Sample':<8} {'Backend':<10} {'Sec':<11} {'fit':>6} {'rmse':>6} {'inliers':>8} {'info_min_eig':>14}")
    print("-" * 75)
    for sample in SAMPLES:
        if not (ROOT / sample).exists():
            continue
        out["per_frame_quality"][sample] = {}
        for bname, bdir, _ in BACKENDS:
            out["per_frame_quality"][sample][bname] = {}
            for sec_short, sec_full in SECONDARIES:
                q = per_frame_quality(sample, bdir, sec_full)
                if q is None:
                    continue
                out["per_frame_quality"][sample][bname][sec_short] = q
                imin = q.get("info_min_eig")
                imin_s = f"{imin:.2e}" if imin is not None else "  --  "
                print(f"{sample:<8} {bname:<10} {sec_short:<11} "
                      f"{q['fitness_mean']:.3f} {q['rmse_mean']:.3f} "
                      f"{q['inliers_mean']:>8.0f} {imin_s:>14}")

    print()
    print("=== B2 aggregated variance (translation_std_m, n_eff) ===")
    print(f"{'Sample':<8} {'Backend':<10} {'Sec':<11} {'std_x':>7} {'std_y':>7} {'std_z':>7} {'n_eff':>7}")
    print("-" * 70)
    for sample in SAMPLES:
        if not (ROOT / sample).exists():
            continue
        out["b2_variance"][sample] = {}
        for bname, bdir, files in BACKENDS:
            calib = load_calib(sample, bdir, files)
            if not calib:
                continue
            out["b2_variance"][sample][bname] = {}
            for sec_short, sec_full in SECONDARIES:
                rec = calib.get(sec_full, {})
                std = rec.get("translation_std_m")
                n_eff = rec.get("n_effective_weighted")
                if std is None or len(std) != 3:
                    continue
                out["b2_variance"][sample][bname][sec_short] = {
                    "std_xyz": std,
                    "n_eff": float(n_eff) if n_eff is not None else None,
                }
                print(f"{sample:<8} {bname:<10} {sec_short:<11} "
                      f"{std[0]:>6.3f} {std[1]:>6.3f} {std[2]:>6.3f} "
                      f"{n_eff or 0:>6.1f}")

    print()
    print("=== Cross-backend spread (no GT — backends agreeing = trustworthy) ===")
    print(f"{'Sample':<8} {'Sec':<11} {'n_bk':>5} {'xyz_std':>22} {'std_norm':>9} {'max_pair_dist':>13}")
    print("-" * 75)
    spreads = cross_backend_spread()
    out["cross_backend_spread"] = spreads
    for sample, by_sec in spreads.items():
        for sec, s in by_sec.items():
            std = s["xyz_std"]
            print(f"{sample:<8} {sec:<11} {s['n_backends']:>5} "
                  f"x{std[0]:.3f} y{std[1]:.3f} z{std[2]:.3f}    "
                  f"{s['xyz_std_norm']:>7.3f}m  {s['max_pairwise_dist']:>10.3f}m")

    out_path = ROOT / "internal_quality_summary.json"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(out, f, indent=2)
    print(f"\nWrote {out_path}")


if __name__ == "__main__":
    main()
