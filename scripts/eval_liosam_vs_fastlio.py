#!/usr/bin/env python3
"""Align LIO-SAM trajectory to FAST-LIO via Umeyama, then report ATE / RPE / Z drift.

Both systems are gravity-aligned (Z is up) but choose an arbitrary initial
yaw + origin. We do a similarity alignment (R, t, s) constrained to s=1
(rigid SE(3)) since both are metric. With ~50° yaw offset observed, this is
purely a frame-of-reference difference — not actual drift.
"""
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_tum(path):
    data = np.loadtxt(path, comments="#")
    return data[:, 0], data[:, 1:4]


def umeyama_align(src, dst, with_scale=False):
    """Find R, t, s s.t. dst ≈ s*R*src + t. Returns (R, t, s)."""
    mu_src = src.mean(0); mu_dst = dst.mean(0)
    src_c = src - mu_src; dst_c = dst - mu_dst
    H = src_c.T @ dst_c / len(src)
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1, 1, d])
    R = Vt.T @ D @ U.T
    if with_scale:
        var_src = (src_c ** 2).sum() / len(src)
        s = (S * np.array([1, 1, d])).sum() / var_src
    else:
        s = 1.0
    t = mu_dst - s * R @ mu_src
    return R, t, s


def time_associate(t_ref, p_ref, t_est, p_est, max_dt=0.06):
    """For each estimated pose, find nearest reference pose within max_dt."""
    pairs = []
    j = 0
    for i in range(len(t_est)):
        while j + 1 < len(t_ref) and abs(t_ref[j+1] - t_est[i]) < abs(t_ref[j] - t_est[i]):
            j += 1
        if abs(t_ref[j] - t_est[i]) < max_dt:
            pairs.append((i, j))
    pairs = np.array(pairs)
    return p_est[pairs[:, 0]], p_ref[pairs[:, 1]], t_est[pairs[:, 0]]


def main():
    fl_path = Path("output/ghcr_run_v3/trajectory.txt")
    ls_path = Path("output/liosam_run/trajectory.txt")
    out_dir = Path("output/liosam_run")

    fl_t, fl_xyz = load_tum(fl_path)
    ls_t, ls_xyz = load_tum(ls_path)
    print(f"FAST-LIO: {len(fl_t)} poses, length="
          f"{np.linalg.norm(np.diff(fl_xyz, axis=0), axis=1).sum():.2f} m")
    print(f"LIO-SAM : {len(ls_t)} keyframes, length="
          f"{np.linalg.norm(np.diff(ls_xyz, axis=0), axis=1).sum():.2f} m")

    # Associate LIO-SAM keyframes with nearest FAST-LIO pose in time
    ls_p, fl_p, t_match = time_associate(fl_t, fl_xyz, ls_t, ls_xyz, max_dt=0.06)
    print(f"Time-associated pairs: {len(ls_p)} (of {len(ls_t)} LIO-SAM keyframes)")

    # Umeyama align (rigid, no scale)
    R, t, s = umeyama_align(ls_p, fl_p, with_scale=False)
    ls_aligned = (s * R @ ls_p.T).T + t
    err = np.linalg.norm(ls_aligned - fl_p, axis=1)
    ate_rmse = float(np.sqrt((err ** 2).mean()))
    ate_mean = float(err.mean())
    ate_max  = float(err.max())
    print(f"\n=== Absolute Trajectory Error (post Umeyama R,t alignment) ===")
    print(f"  RMSE : {ate_rmse:.3f} m")
    print(f"  mean : {ate_mean:.3f} m")
    print(f"  max  : {ate_max:.3f} m")
    print(f"  yaw between frames: {np.degrees(np.arctan2(R[1,0], R[0,0])):.2f}°")

    # Z-drift comparison (both methods report Z relative to start)
    print(f"\n=== Z-drift comparison ===")
    print(f"  FAST-LIO Z: [{fl_xyz[:,2].min():.3f}, {fl_xyz[:,2].max():.3f}] m  "
          f"std={fl_xyz[:,2].std():.3f}")
    print(f"  LIO-SAM  Z: [{ls_xyz[:,2].min():.3f}, {ls_xyz[:,2].max():.3f}] m  "
          f"std={ls_xyz[:,2].std():.3f}")

    # Relative pose error per 1 m sub-segment (translation RPE)
    def rpe(xyz, t, segment_m=10.0):
        seg_errs = []
        cum_dist = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(xyz, axis=0), axis=1))])
        for i in range(len(xyz)):
            # find first j with cum_dist[j] - cum_dist[i] >= segment_m
            j = np.searchsorted(cum_dist - cum_dist[i], segment_m)
            if j >= len(xyz):
                continue
            seg_errs.append((i, j, cum_dist[j] - cum_dist[i]))
        return seg_errs

    # Apply RPE on aligned LIO-SAM vs FAST-LIO subsampled to matched poses
    diffs_ls = np.diff(ls_aligned, axis=0); diffs_fl = np.diff(fl_p, axis=0)
    seg_rel_err = np.linalg.norm(diffs_ls - diffs_fl, axis=1) / np.maximum(np.linalg.norm(diffs_fl, axis=1), 1e-3)
    print(f"\n=== Per-segment relative drift (LIO-SAM vs FAST-LIO, between matched poses) ===")
    print(f"  median |err|/|fl| : {np.median(seg_rel_err)*100:.2f}%")
    print(f"  mean   |err|/|fl| : {seg_rel_err.mean()*100:.2f}%")
    print(f"  90th-pctile        : {np.percentile(seg_rel_err, 90)*100:.2f}%")

    # Plot aligned trajectories
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.plot(fl_xyz[:, 0], fl_xyz[:, 1], "b-", lw=1.5, label=f"FAST-LIO ({len(fl_xyz)} poses)")
    ax.plot(ls_aligned[:, 0], ls_aligned[:, 1], "r--", lw=1.5,
            label=f"LIO-SAM aligned ({len(ls_aligned)} kfs)")
    ax.scatter(fl_xyz[0, 0], fl_xyz[0, 1], c="green", s=80, zorder=5, label="start")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.axis("equal")
    ax.set_title(f"After SE(3) alignment: ATE RMSE = {ate_rmse:.3f} m")
    ax.grid(True, alpha=0.3); ax.legend()
    fig.tight_layout(); fig.savefig(out_dir / "compare_aligned_topdown.png", dpi=120); plt.close(fig)

    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(t_match - t_match[0], err, "r-", lw=1)
    ax.fill_between(t_match - t_match[0], 0, err, alpha=0.2, color="red")
    ax.set_xlabel("time (s)"); ax.set_ylabel("|aligned LIO-SAM − FAST-LIO| (m)")
    ax.set_title(f"Position error over time  |  RMSE {ate_rmse:.3f} m, max {ate_max:.3f} m")
    ax.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig(out_dir / "compare_error_vs_time.png", dpi=120); plt.close(fig)
    print(f"\nWrote: {out_dir}/compare_aligned_topdown.png, compare_error_vs_time.png")


if __name__ == "__main__":
    main()
