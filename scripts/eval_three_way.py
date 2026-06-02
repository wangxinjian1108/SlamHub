#!/usr/bin/env python3
"""Three-way trajectory comparison: FAST-LIO vs KISS-ICP vs LIO-SAM.

All three trajectories are loaded in TUM format and aligned to FAST-LIO via
Umeyama SE(3) (no scale, both sides metric). Reports:
  - per-method path length, frame count, Z drift std
  - ATE (RMSE/mean/max) of KISS-ICP and LIO-SAM vs FAST-LIO
  - RPE per 1m sub-segment
  - top-down overlay + per-axis time series + error vs time
"""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_tum(path):
    data = np.loadtxt(path, comments="#")
    return data[:, 0], data[:, 1:4]


def umeyama_se3(src, dst):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    sc, dc = src - mu_s, dst - mu_d
    H = sc.T @ dc / len(src)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    t = mu_d - R @ mu_s
    return R, t


def time_associate(t_ref, p_ref, t_est, p_est, max_dt=0.06):
    pairs, j = [], 0
    for i in range(len(t_est)):
        while j + 1 < len(t_ref) and abs(t_ref[j+1]-t_est[i]) < abs(t_ref[j]-t_est[i]):
            j += 1
        if abs(t_ref[j] - t_est[i]) < max_dt:
            pairs.append((i, j))
    pairs = np.array(pairs)
    return pairs[:, 0], pairs[:, 1]


def length(xyz):
    return float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum())


def main():
    out_dir = Path("output/three_way_compare")
    out_dir.mkdir(exist_ok=True, parents=True)
    inputs = {
        "FAST-LIO": Path("output/ghcr_run_v3/trajectory.txt"),
        "KISS-ICP": Path("output/kiss_icp_run/trajectory_imu.txt"),
        "LIO-SAM":  Path("output/liosam_run/trajectory.txt"),
        "GenZ-ICP": Path("output/genz_icp_run/trajectory.txt"),
        "MAD-ICP":  Path("output/mad_icp_run/trajectory.txt"),
    }
    raw = {name: load_tum(p) for name, p in inputs.items()}

    # Stats on raw (un-aligned) trajectories
    print(f"{'Backend':<10} {'frames':>7}  {'length(m)':>10}  {'Z range (m)':>22}  {'Z std':>7}")
    for name, (t, xyz) in raw.items():
        L = length(xyz)
        zr = (xyz[:, 2].min(), xyz[:, 2].max())
        print(f"{name:<10} {len(t):>7}  {L:>10.2f}  [{zr[0]:>7.3f},{zr[1]:>7.3f}]      {xyz[:,2].std():>5.3f}")

    # Use FAST-LIO as reference, align KISS-ICP / LIO-SAM / GenZ-ICP / MAD-ICP to it
    fl_t, fl_xyz = raw["FAST-LIO"]
    aligned = {"FAST-LIO": fl_xyz}
    metrics = {}
    for name in ("KISS-ICP", "LIO-SAM", "GenZ-ICP", "MAD-ICP"):
        if name not in raw:
            continue
        t_e, xyz_e = raw[name]
        idx_e, idx_r = time_associate(fl_t, fl_xyz, t_e, xyz_e)
        if len(idx_e) < 3:
            print(f"\n{name}: too few time-matched pairs ({len(idx_e)}); skipping ATE.")
            continue
        R, T = umeyama_se3(xyz_e[idx_e], fl_xyz[idx_r])
        xyz_aligned_full = (R @ xyz_e.T).T + T
        aligned[name] = xyz_aligned_full
        err = np.linalg.norm(xyz_aligned_full[idx_e] - fl_xyz[idx_r], axis=1)
        rmse = float(np.sqrt((err**2).mean()))
        metrics[name] = {
            "n_pairs": len(idx_e),
            "ate_rmse": rmse,
            "ate_mean": float(err.mean()),
            "ate_max":  float(err.max()),
            "yaw_to_fl_deg": float(np.degrees(np.arctan2(R[1,0], R[0,0]))),
            "err": err,
            "t":   fl_t[idx_r] - fl_t[0],
        }
        # Per-segment relative drift between consecutive matched poses
        diffs_e = np.diff(xyz_aligned_full[idx_e], axis=0)
        diffs_r = np.diff(fl_xyz[idx_r], axis=0)
        rel = np.linalg.norm(diffs_e - diffs_r, axis=1) / np.maximum(np.linalg.norm(diffs_r, axis=1), 1e-3)
        metrics[name]["rpe_med_pct"] = float(np.median(rel) * 100)
        metrics[name]["rpe_p90_pct"] = float(np.percentile(rel, 90) * 100)

    print(f"\n{'Backend':<10} {'pairs':>6}  {'ATE rmse':>9}  {'mean':>6}  {'max':>6}  {'yaw_to_FL':>9}  {'RPE med%':>8}  {'RPE p90%':>8}")
    for name, m in metrics.items():
        print(f"{name:<10} {m['n_pairs']:>6}  {m['ate_rmse']:>8.3f}m {m['ate_mean']:>5.3f}m {m['ate_max']:>5.3f}m  {m['yaw_to_fl_deg']:>7.2f}°  {m['rpe_med_pct']:>7.2f}%  {m['rpe_p90_pct']:>7.2f}%")

    # --- Plots ---
    colors = {"FAST-LIO": "tab:blue", "KISS-ICP": "tab:green",
              "LIO-SAM": "tab:red",  "GenZ-ICP": "tab:purple",
              "MAD-ICP": "tab:orange"}
    # matplotlib's plot() rejects tuple dash specs in the fmt string slot,
    # so keep all styles as named strings — uniqueness is good enough for
    # 5 lines.
    styles = {"FAST-LIO": "-", "KISS-ICP": "-.",
              "LIO-SAM":  "--", "GenZ-ICP": ":",
              "MAD-ICP":  "-"}

    fig, ax = plt.subplots(figsize=(11, 8))
    for name, xyz in aligned.items():
        ax.plot(xyz[:, 0], xyz[:, 1], styles[name], lw=1.6, color=colors[name],
                label=f"{name} ({len(xyz)} pts)")
    ax.scatter(fl_xyz[0, 0], fl_xyz[0, 1], c="black", s=80, marker="o", zorder=5, label="start")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.axis("equal")
    ax.set_title("5-way trajectory overlay (others SE(3)-aligned to FAST-LIO)")
    ax.grid(True, alpha=0.3); ax.legend()
    fig.tight_layout(); fig.savefig(out_dir / "three_way_topdown.png", dpi=120); plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    times = {}
    for name, (t, _) in raw.items():
        times[name] = t - t.min()
    for i, lbl in enumerate("xyz"):
        for name, xyz in aligned.items():
            axes[i].plot(times[name][:len(xyz)], xyz[:, i], styles[name], lw=1.2,
                         color=colors[name], label=name)
        axes[i].set_ylabel(f"{lbl} (m)"); axes[i].grid(True, alpha=0.3)
        if i == 0:
            axes[i].legend(ncol=3)
    axes[2].set_xlabel("time since start (s)")
    fig.suptitle("Per-axis trajectory (after SE(3) align to FAST-LIO)")
    fig.tight_layout(); fig.savefig(out_dir / "three_way_xyz.png", dpi=120); plt.close(fig)

    fig, ax = plt.subplots(figsize=(12, 5))
    for name, m in metrics.items():
        ax.plot(m["t"], m["err"], styles[name], color=colors[name], lw=1.2,
                label=f"{name}  RMSE={m['ate_rmse']:.3f}m")
    ax.set_xlabel("time (s)"); ax.set_ylabel("position error vs FAST-LIO (m)")
    ax.set_title("ATE over time — KISS-ICP and LIO-SAM vs FAST-LIO")
    ax.grid(True, alpha=0.3); ax.legend()
    fig.tight_layout(); fig.savefig(out_dir / "three_way_error_time.png", dpi=120); plt.close(fig)

    print(f"\nWrote: {out_dir}/three_way_{{topdown,xyz,error_time}}.png")

    # Save numeric summary
    summary = {
        "raw_lengths_m": {name: length(raw[name][1]) for name in raw},
        "raw_z_std_m":   {name: float(raw[name][1][:, 2].std()) for name in raw},
        "raw_n_frames":  {name: int(len(raw[name][0])) for name in raw},
        "metrics_vs_fastlio": {name: {k: v for k, v in m.items() if k not in ("err", "t")}
                               for name, m in metrics.items()},
    }
    import json
    with open(out_dir / "three_way_summary.json", "w") as f:
        json.dump(summary, f, indent=2)
    print(f"Wrote: {out_dir}/three_way_summary.json")


if __name__ == "__main__":
    main()
