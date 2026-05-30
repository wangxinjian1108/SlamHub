#!/usr/bin/env python3
"""Compare FAST-LIO trajectory against LIDAR_TO_MAP reference.

LIDAR_TO_MAP/<idx>_<ts_ns>.txt holds T_map_baselink (4x4) — the recording's
own baselink pose in some absolute map frame (likely UTM-anchored).

FAST-LIO trajectory.txt is T_world_imu in a local map frame that starts near
origin. To compare:
1. Compose T_world_baselink = T_world_imu @ T_imu_baselink  (T_imu_baselink
   from application.yaml: invert(baselink->imu))
2. Match each LIDAR_TO_MAP entry to nearest trajectory pose by timestamp.
3. Rigid SE(3) align the SLAM positions to the reference (Umeyama-style,
   no scaling): solve R,t minimizing ||R @ p_slam + t - p_ref||.
4. Report ATE (RMS translation error), per-axis bias/std, rotation angle
   error mean/std, and plot top-down trajectories + error-vs-time.

Usage:
    python3 scripts/compare_with_lidar_to_map.py \
        --slam-trajectory output/ghcr_run_v3/trajectory.txt \
        --reference-dir <recording>/LIDAR_TO_MAP \
        --application-yaml <recording>/application.yaml \
        --output-dir output/ghcr_run_v3
"""
import argparse
import sys
from pathlib import Path
import numpy as np
import yaml
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).parent))
from common.io import read_trajectory_tum
from common.transform import (
    euler_to_matrix, quaternion_to_matrix, make_homogeneous, invert_transform,
)


def load_baselink_to_imu(application_yaml):
    cfg = yaml.safe_load(open(application_yaml))
    for cal in cfg["vehicle"]["calibration"]["sensor_calibration"]:
        if cal["source"] == "FRAME_GNSS_IMU":
            t = cal["transformation"]
            R = euler_to_matrix(t[3], t[4], t[5])
            return make_homogeneous(R, np.array(t[:3]))
    return np.eye(4)


def load_reference(ref_dir):
    """Return ts_ns sorted array + list of 4x4 transforms."""
    files = sorted(ref_dir.glob("*.txt"))
    ts_list, T_list = [], []
    for f in files:
        # Filename like 0000_000_1751437740202796064.txt
        stem = f.stem
        ts_ns = int(stem.split("_")[-1])
        T = np.loadtxt(f)
        if T.shape != (4, 4):
            continue
        ts_list.append(ts_ns)
        T_list.append(T)
    return np.array(ts_list), T_list


def load_slam(traj_path, T_imu_baselink):
    poses = read_trajectory_tum(traj_path)
    ts = (poses[:, 0] * 1e9).astype(np.int64)
    T_list = []
    for r in poses:
        _, tx, ty, tz, qx, qy, qz, qw = r
        T_wi = make_homogeneous(quaternion_to_matrix(qx, qy, qz, qw),
                                 np.array([tx, ty, tz]))
        # T_world_baselink = T_world_imu @ T_imu_baselink
        T_list.append(T_wi @ T_imu_baselink)
    return ts, T_list


def match_by_timestamp(ts_ref, ts_slam, tol_ns=60_000_000):
    """For each ref index, find nearest slam index within tol_ns."""
    out = []  # (ref_idx, slam_idx)
    for i, t in enumerate(ts_ref):
        j = int(np.argmin(np.abs(ts_slam - t)))
        if abs(int(ts_slam[j]) - int(t)) <= tol_ns:
            out.append((i, j))
    return out


def rigid_align(P, Q):
    """Find R, t such that R @ P + t ≈ Q. P, Q are (N, 3). Returns R(3,3), t(3,)."""
    Pc = P.mean(0); Qc = Q.mean(0)
    Pp = P - Pc; Qp = Q - Qc
    H = Pp.T @ Qp
    U, S, Vt = np.linalg.svd(H)
    d = np.linalg.det(Vt.T @ U.T)
    D = np.diag([1, 1, 1 if d > 0 else -1])
    R = Vt.T @ D @ U.T
    t = Qc - R @ Pc
    return R, t


def rot_angle_deg(R):
    cos_th = (np.trace(R) - 1) / 2.0
    cos_th = np.clip(cos_th, -1.0, 1.0)
    return np.degrees(np.arccos(cos_th))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--slam-trajectory", type=Path, required=True)
    p.add_argument("--reference-dir", type=Path, required=True)
    p.add_argument("--application-yaml", type=Path, required=True)
    p.add_argument("--output-dir", type=Path, required=True)
    p.add_argument("--tol-ms", type=float, default=60.0,
                   help="timestamp matching tolerance, ms")
    args = p.parse_args()

    T_baselink_imu = load_baselink_to_imu(args.application_yaml)
    T_imu_baselink = invert_transform(T_baselink_imu)

    ts_ref, T_ref = load_reference(args.reference_dir)
    print(f"Reference: {len(ts_ref)} poses, ts range "
          f"{ts_ref[0]/1e9:.3f}..{ts_ref[-1]/1e9:.3f}")
    ts_slam, T_slam = load_slam(args.slam_trajectory, T_imu_baselink)
    print(f"SLAM     : {len(ts_slam)} poses, ts range "
          f"{ts_slam[0]/1e9:.3f}..{ts_slam[-1]/1e9:.3f}")

    pairs = match_by_timestamp(ts_ref, ts_slam, int(args.tol_ms * 1e6))
    print(f"Matched  : {len(pairs)} pairs within ±{args.tol_ms}ms")
    if len(pairs) < 10:
        print("Too few matches.")
        sys.exit(1)

    P_slam = np.array([T_slam[j][:3, 3] for _, j in pairs])
    P_ref = np.array([T_ref[i][:3, 3] for i, _ in pairs])
    R_align, t_align = rigid_align(P_slam, P_ref)
    print(f"Alignment translation: {t_align}")
    print(f"Alignment rotation angle: {rot_angle_deg(R_align):.4f}°")

    # Transform SLAM trajectory into reference frame
    P_slam_a = (R_align @ P_slam.T).T + t_align
    err = P_slam_a - P_ref
    err_norm = np.linalg.norm(err, axis=1)
    ate_rms = float(np.sqrt((err_norm ** 2).mean()))
    ate_mean = float(err_norm.mean())
    ate_med = float(np.median(err_norm))

    # Rotation errors
    rot_errs = []
    for (i, j) in pairs:
        R_ref = T_ref[i][:3, :3]
        R_slam = R_align @ T_slam[j][:3, :3]
        R_delta = R_ref.T @ R_slam
        rot_errs.append(rot_angle_deg(R_delta))
    rot_errs = np.array(rot_errs)

    times = np.array([(ts_ref[i] - ts_ref[0]) / 1e9 for i, _ in pairs])

    print("\n=== Trajectory comparison ===")
    print(f"  ATE RMS : {ate_rms:.4f} m")
    print(f"  ATE mean: {ate_mean:.4f} m")
    print(f"  ATE med : {ate_med:.4f} m")
    print(f"  ATE max : {err_norm.max():.4f} m")
    print(f"  per-axis bias (slam-ref, m): "
          f"dx={err[:,0].mean():+.4f}±{err[:,0].std():.4f} "
          f"dy={err[:,1].mean():+.4f}±{err[:,1].std():.4f} "
          f"dz={err[:,2].mean():+.4f}±{err[:,2].std():.4f}")
    print(f"  Rot err mean: {rot_errs.mean():.4f}° std {rot_errs.std():.4f}° "
          f"med {np.median(rot_errs):.4f}° max {rot_errs.max():.4f}°")

    # ---- Plots ----
    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)

    # Plot 1: top-down trajectories overlay (in REF frame)
    fig, ax = plt.subplots(figsize=(12, 12))
    ax.plot(P_ref[:, 0], P_ref[:, 1], "b-", lw=1.5, label="LIDAR_TO_MAP reference")
    ax.plot(P_slam_a[:, 0], P_slam_a[:, 1], "r--", lw=1.5, label="FAST-LIO (aligned)")
    ax.scatter([P_ref[0, 0]], [P_ref[0, 1]], c="lime", s=120, marker="o",
               edgecolors="k", label="start", zorder=5)
    ax.scatter([P_ref[-1, 0]], [P_ref[-1, 1]], c="orange", s=120, marker="*",
               edgecolors="k", label="end", zorder=5)
    ax.set_xlabel("X (m, reference frame)"); ax.set_ylabel("Y (m, reference frame)")
    ax.set_aspect("equal"); ax.grid(alpha=0.3)
    ax.set_title(f"Trajectory overlay — ATE RMS = {ate_rms:.3f} m  ({len(pairs)} matched poses)")
    ax.legend()
    fig.savefig(out / "compare_traj_topdown.png", dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"\nSaved {out / 'compare_traj_topdown.png'}")

    # Plot 2: error vs time
    fig, axs = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    axs[0].plot(times, err_norm, "k-", lw=0.8)
    axs[0].axhline(ate_rms, color="r", ls="--", lw=0.8, label=f"RMS={ate_rms:.3f}m")
    axs[0].set_ylabel("‖Δpos‖ (m)"); axs[0].grid(alpha=0.3); axs[0].legend()
    axs[1].plot(times, err[:, 0], label="dx")
    axs[1].plot(times, err[:, 1], label="dy")
    axs[1].plot(times, err[:, 2], label="dz")
    axs[1].set_ylabel("axis error (m)"); axs[1].grid(alpha=0.3); axs[1].legend()
    axs[2].plot(times, rot_errs, "k-", lw=0.8)
    axs[2].set_ylabel("rotation angle err (°)"); axs[2].set_xlabel("time (s)")
    axs[2].grid(alpha=0.3)
    fig.suptitle("FAST-LIO vs LIDAR_TO_MAP — translation & rotation error over time")
    fig.savefig(out / "compare_traj_error.png", dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved {out / 'compare_traj_error.png'}")

    # Plot 3: per-axis position vs time (overlay)
    fig, axs = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for i, lbl in enumerate("XYZ"):
        axs[i].plot(times, P_ref[:, i], "b-", lw=1.0, label="reference")
        axs[i].plot(times, P_slam_a[:, i], "r--", lw=1.0, label="SLAM aligned")
        axs[i].set_ylabel(f"{lbl} (m)"); axs[i].grid(alpha=0.3); axs[i].legend()
    axs[2].set_xlabel("time (s)")
    fig.suptitle("Position components: reference (blue) vs FAST-LIO aligned (red dashed)")
    fig.savefig(out / "compare_traj_xyz.png", dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved {out / 'compare_traj_xyz.png'}")

    # Save summary YAML
    summary = {
        "matched_pairs": len(pairs),
        "tolerance_ms": float(args.tol_ms),
        "alignment_translation_m": [float(x) for x in t_align],
        "alignment_rotation_deg": float(rot_angle_deg(R_align)),
        "ate_rms_m": ate_rms,
        "ate_mean_m": ate_mean,
        "ate_median_m": ate_med,
        "ate_max_m": float(err_norm.max()),
        "axis_bias_m": {
            "dx": float(err[:, 0].mean()),
            "dy": float(err[:, 1].mean()),
            "dz": float(err[:, 2].mean()),
        },
        "axis_std_m": {
            "dx": float(err[:, 0].std()),
            "dy": float(err[:, 1].std()),
            "dz": float(err[:, 2].std()),
        },
        "rotation_error_deg": {
            "mean": float(rot_errs.mean()),
            "std": float(rot_errs.std()),
            "median": float(np.median(rot_errs)),
            "max": float(rot_errs.max()),
        },
    }
    yaml_out = out / "compare_traj_summary.yaml"
    with open(yaml_out, "w") as f:
        yaml.dump(summary, f, default_flow_style=None, sort_keys=False)
    print(f"Saved {yaml_out}")


if __name__ == "__main__":
    main()
