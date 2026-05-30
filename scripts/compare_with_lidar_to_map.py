#!/usr/bin/env python3
"""Compare FAST-LIO trajectory against LIDAR_TO_MAP reference, four ways.

LIDAR_TO_MAP/<idx>_<ts_ns>.txt holds T_map_baselink (4x4) at the lidar frame
timestamps. FAST-LIO trajectory.txt is T_world_imu; we compose
T_world_baselink = T_world_imu @ T_imu_baselink with T_imu_baselink from
application.yaml.

We evaluate four combinations:
    1) NEAREST + GLOBAL    — original baseline
    2) NEAREST + FIRSTFRAME — first matched pair forced to identity, then
                              compare downstream drift
    3) INTERP   + GLOBAL    — SLAM poses interpolated to reference timestamps
                              (SLERP rotation, linear translation) before
                              global SE(3) alignment
    4) INTERP   + FIRSTFRAME

Each variant reports ATE (RMS / mean / median / max), per-axis bias+std,
and rotation angle error stats. A combined report YAML, comparison PNGs,
and a printed table are produced.
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
    euler_to_matrix, quaternion_to_matrix, matrix_to_quaternion,
    make_homogeneous, invert_transform,
)


# ---------- IO ----------

def load_baselink_to_imu(application_yaml):
    cfg = yaml.safe_load(open(application_yaml))
    for cal in cfg["vehicle"]["calibration"]["sensor_calibration"]:
        if cal["source"] == "FRAME_GNSS_IMU":
            t = cal["transformation"]
            R = euler_to_matrix(t[3], t[4], t[5])
            return make_homogeneous(R, np.array(t[:3]))
    return np.eye(4)


def load_reference(ref_dir):
    files = sorted(ref_dir.glob("*.txt"))
    ts_list, T_list = [], []
    for f in files:
        ts_ns = int(f.stem.split("_")[-1])
        T = np.loadtxt(f)
        if T.shape != (4, 4):
            continue
        ts_list.append(ts_ns)
        T_list.append(T)
    return np.array(ts_list), np.array(T_list)


def load_slam(traj_path, T_imu_baselink):
    poses = read_trajectory_tum(traj_path)
    ts = (poses[:, 0] * 1e9).astype(np.int64)
    T_list = []
    for r in poses:
        _, tx, ty, tz, qx, qy, qz, qw = r
        T_wi = make_homogeneous(quaternion_to_matrix(qx, qy, qz, qw),
                                 np.array([tx, ty, tz]))
        T_list.append(T_wi @ T_imu_baselink)
    return ts, np.array(T_list)


# ---------- Matching ----------

def match_nearest(ts_ref, ts_slam, tol_ns):
    """Per ref idx -> nearest slam idx within tol_ns."""
    pairs = []
    for i, t in enumerate(ts_ref):
        j = int(np.argmin(np.abs(ts_slam - t)))
        if abs(int(ts_slam[j]) - int(t)) <= tol_ns:
            pairs.append((i, j))
    return pairs


def slerp(q0, q1, u):
    """Quaternion slerp; q* are (qx,qy,qz,qw)."""
    if np.dot(q0, q1) < 0:
        q1 = -q1
    dot = float(np.clip(np.dot(q0, q1), -1.0, 1.0))
    if dot > 0.9995:
        q = q0 + u * (q1 - q0)
        return q / np.linalg.norm(q)
    th0 = np.arccos(dot); th = th0 * u
    s = np.sin(th0)
    a = np.sin(th0 - th) / s; b = np.sin(th) / s
    return a * q0 + b * q1


def interp_slam_to_ref(ts_ref, ts_slam, T_slam, max_gap_ns):
    """For each ref ts, interpolate SLAM pose (SLERP rotation, linear translation).
    Accept whenever the ref ts is bracketed by two SLAM samples whose gap is
    ≤ max_gap_ns (typical sensor period × 2). Boundary refs that fall outside
    the SLAM time range are dropped."""
    N = len(ts_ref)
    T_out = np.empty((N, 4, 4))
    valid = np.zeros(N, dtype=bool)
    for i, t in enumerate(ts_ref):
        j = int(np.searchsorted(ts_slam, t))
        if j <= 0 or j >= len(ts_slam):
            continue
        t0 = int(ts_slam[j - 1]); t1 = int(ts_slam[j])
        if t1 - t0 > max_gap_ns or t1 == t0:
            continue
        u = (int(t) - t0) / (t1 - t0)
        p = (1 - u) * T_slam[j - 1][:3, 3] + u * T_slam[j][:3, 3]
        q0 = np.array(matrix_to_quaternion(T_slam[j - 1][:3, :3]))
        q1 = np.array(matrix_to_quaternion(T_slam[j][:3, :3]))
        q = slerp(q0, q1, u)
        R = quaternion_to_matrix(*q)
        T_out[i] = make_homogeneous(R, p)
        valid[i] = True
    return T_out, valid


# ---------- Alignment ----------

def rigid_align(P, Q):
    """R, t s.t. R @ P + t ≈ Q. P, Q (N, 3). Returns R (3,3), t (3,)."""
    Pc = P.mean(0); Qc = Q.mean(0)
    H = (P - Pc).T @ (Q - Qc)
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1, 1, 1 if np.linalg.det(Vt.T @ U.T) > 0 else -1])
    R = Vt.T @ D @ U.T
    return R, Qc - R @ Pc


def first_frame_align(T_ref0, T_slam0):
    """Find R, t such that R @ pos_slam[0] + t = pos_ref[0] AND
    the SE(3) rotation aligns slam[0] to ref[0]."""
    delta = T_ref0 @ invert_transform(T_slam0)  # (slam_frame -> ref_frame) for pose 0
    return delta[:3, :3], delta[:3, 3]


def apply_R_t(P, R, t):
    return (R @ P.T).T + t


def rot_angle_deg(R):
    cos_th = (np.trace(R) - 1) / 2.0
    return float(np.degrees(np.arccos(np.clip(cos_th, -1.0, 1.0))))


# ---------- Evaluation ----------

def evaluate(P_slam_aligned, P_ref, R_ref_list, R_slam_aligned_list):
    err = P_slam_aligned - P_ref
    err_norm = np.linalg.norm(err, axis=1)
    rot_errs = []
    for R_r, R_s in zip(R_ref_list, R_slam_aligned_list):
        rot_errs.append(rot_angle_deg(R_r.T @ R_s))
    rot_errs = np.array(rot_errs)
    return dict(
        n=len(err_norm),
        ate_rms=float(np.sqrt((err_norm ** 2).mean())),
        ate_mean=float(err_norm.mean()),
        ate_median=float(np.median(err_norm)),
        ate_max=float(err_norm.max()),
        dx_mean=float(err[:, 0].mean()), dx_std=float(err[:, 0].std()),
        dy_mean=float(err[:, 1].mean()), dy_std=float(err[:, 1].std()),
        dz_mean=float(err[:, 2].mean()), dz_std=float(err[:, 2].std()),
        rot_mean=float(rot_errs.mean()), rot_std=float(rot_errs.std()),
        rot_median=float(np.median(rot_errs)), rot_max=float(rot_errs.max()),
        err=err, err_norm=err_norm, rot_errs=rot_errs,
    )


# ---------- Driver ----------

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--slam-trajectory", type=Path, required=True)
    p.add_argument("--reference-dir", type=Path, required=True)
    p.add_argument("--application-yaml", type=Path, required=True)
    p.add_argument("--output-dir", type=Path, required=True)
    p.add_argument("--tol-ms", type=float, default=60.0)
    args = p.parse_args()

    T_baselink_imu = load_baselink_to_imu(args.application_yaml)
    T_imu_baselink = invert_transform(T_baselink_imu)

    ts_ref, T_ref = load_reference(args.reference_dir)
    ts_slam, T_slam = load_slam(args.slam_trajectory, T_imu_baselink)
    tol_ns = int(args.tol_ms * 1e6)
    print(f"Ref poses: {len(ts_ref)}, SLAM poses: {len(ts_slam)}, tol ±{args.tol_ms}ms")

    # ---- 4 variants ----
    variants = {}

    # NEAREST
    pairs = match_nearest(ts_ref, ts_slam, tol_ns)
    if pairs:
        ridx, sidx = zip(*pairs)
        ridx = np.array(ridx); sidx = np.array(sidx)
        T_slam_m = T_slam[sidx]
        T_ref_m = T_ref[ridx]
        ts_m = ts_ref[ridx]

        # global align
        R, t = rigid_align(T_slam_m[:, :3, 3], T_ref_m[:, :3, 3])
        P_slam_a = apply_R_t(T_slam_m[:, :3, 3], R, t)
        R_slam_a = [R @ T_slam_m[i, :3, :3] for i in range(len(T_slam_m))]
        v = evaluate(P_slam_a, T_ref_m[:, :3, 3],
                     [T_ref_m[i, :3, :3] for i in range(len(T_ref_m))], R_slam_a)
        v["align_R"], v["align_t"] = R, t
        v["ts"] = ts_m
        v["P_ref"] = T_ref_m[:, :3, 3]; v["P_slam_a"] = P_slam_a
        variants["nearest_global"] = v

        # first-frame align
        R, t = first_frame_align(T_ref_m[0], T_slam_m[0])
        P_slam_a = apply_R_t(T_slam_m[:, :3, 3], R, t)
        R_slam_a = [R @ T_slam_m[i, :3, :3] for i in range(len(T_slam_m))]
        v = evaluate(P_slam_a, T_ref_m[:, :3, 3],
                     [T_ref_m[i, :3, :3] for i in range(len(T_ref_m))], R_slam_a)
        v["align_R"], v["align_t"] = R, t
        v["ts"] = ts_m
        v["P_ref"] = T_ref_m[:, :3, 3]; v["P_slam_a"] = P_slam_a
        variants["nearest_firstframe"] = v

    # INTERP — allow brackets up to 2x the nominal lidar period (100ms @ 10Hz)
    T_slam_i, valid = interp_slam_to_ref(ts_ref, ts_slam, T_slam,
                                          max_gap_ns=200_000_000)
    if valid.any():
        T_slam_m = T_slam_i[valid]
        T_ref_m = T_ref[valid]
        ts_m = ts_ref[valid]

        R, t = rigid_align(T_slam_m[:, :3, 3], T_ref_m[:, :3, 3])
        P_slam_a = apply_R_t(T_slam_m[:, :3, 3], R, t)
        R_slam_a = [R @ T_slam_m[i, :3, :3] for i in range(len(T_slam_m))]
        v = evaluate(P_slam_a, T_ref_m[:, :3, 3],
                     [T_ref_m[i, :3, :3] for i in range(len(T_ref_m))], R_slam_a)
        v["align_R"], v["align_t"] = R, t
        v["ts"] = ts_m
        v["P_ref"] = T_ref_m[:, :3, 3]; v["P_slam_a"] = P_slam_a
        variants["interp_global"] = v

        R, t = first_frame_align(T_ref_m[0], T_slam_m[0])
        P_slam_a = apply_R_t(T_slam_m[:, :3, 3], R, t)
        R_slam_a = [R @ T_slam_m[i, :3, :3] for i in range(len(T_slam_m))]
        v = evaluate(P_slam_a, T_ref_m[:, :3, 3],
                     [T_ref_m[i, :3, :3] for i in range(len(T_ref_m))], R_slam_a)
        v["align_R"], v["align_t"] = R, t
        v["ts"] = ts_m
        v["P_ref"] = T_ref_m[:, :3, 3]; v["P_slam_a"] = P_slam_a
        variants["interp_firstframe"] = v

    # ---- Print table ----
    cols = ["nearest_global", "interp_global", "nearest_firstframe", "interp_firstframe"]
    headers = ["nearest+global", "interp+global", "nearest+first", "interp+first"]
    print(f"\n{'metric':>22} | " + " | ".join(f"{h:>15}" for h in headers))
    print("-" * 90)
    rows = [
        ("n_pairs", "n", "{}"),
        ("ATE RMS (m)", "ate_rms", "{:.4f}"),
        ("ATE mean (m)", "ate_mean", "{:.4f}"),
        ("ATE median (m)", "ate_median", "{:.4f}"),
        ("ATE max (m)", "ate_max", "{:.4f}"),
        ("dx std (m)", "dx_std", "{:.4f}"),
        ("dy std (m)", "dy_std", "{:.4f}"),
        ("dz std (m)", "dz_std", "{:.4f}"),
        ("dx bias (m)", "dx_mean", "{:+.4f}"),
        ("dy bias (m)", "dy_mean", "{:+.4f}"),
        ("dz bias (m)", "dz_mean", "{:+.4f}"),
        ("rot mean (deg)", "rot_mean", "{:.4f}"),
        ("rot max  (deg)", "rot_max", "{:.4f}"),
    ]
    for label, key, fmt in rows:
        row = f"{label:>22} | "
        for c in cols:
            v = variants.get(c)
            row += f"{fmt.format(v[key]) if v else '-':>15} | "
        print(row.rstrip(" |"))

    # ---- Save summary YAML ----
    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)
    summary = {}
    for c in cols:
        v = variants.get(c)
        if v is None: continue
        summary[c] = {
            "n_pairs": int(v["n"]),
            "alignment_translation_m": [float(x) for x in v["align_t"]],
            "alignment_rotation_deg": rot_angle_deg(v["align_R"]),
            "ate_rms_m": v["ate_rms"], "ate_mean_m": v["ate_mean"],
            "ate_median_m": v["ate_median"], "ate_max_m": v["ate_max"],
            "axis_bias_m": dict(dx=v["dx_mean"], dy=v["dy_mean"], dz=v["dz_mean"]),
            "axis_std_m":  dict(dx=v["dx_std"],  dy=v["dy_std"],  dz=v["dz_std"]),
            "rotation_error_deg": dict(mean=v["rot_mean"], std=v["rot_std"],
                                       median=v["rot_median"], max=v["rot_max"]),
        }
    with open(out / "compare_traj_summary.yaml", "w") as f:
        yaml.dump(summary, f, default_flow_style=None, sort_keys=False)
    print(f"\nSaved {out / 'compare_traj_summary.yaml'}")

    # ---- Plot: error vs time, 4 variants overlaid ----
    fig, axs = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
    for c, h, color in zip(cols, headers,
                           ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]):
        v = variants.get(c)
        if v is None: continue
        t = (v["ts"] - v["ts"][0]) / 1e9
        axs[0].plot(t, v["err_norm"], lw=1.0, label=h, color=color)
        axs[1].plot(t, v["rot_errs"], lw=1.0, label=h, color=color)
    axs[0].set_ylabel("‖Δpos‖ (m)"); axs[0].grid(alpha=0.3); axs[0].legend()
    axs[1].set_ylabel("rotation err (°)"); axs[1].set_xlabel("time (s)")
    axs[1].grid(alpha=0.3); axs[1].legend()
    fig.suptitle("FAST-LIO vs LIDAR_TO_MAP error — 4 variants")
    fig.savefig(out / "compare_traj_error_4way.png", dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved {out / 'compare_traj_error_4way.png'}")

    # ---- Plot: topdown overlay for each alignment ----
    fig, axs = plt.subplots(1, 2, figsize=(20, 10))
    for ax, key, h in [(axs[0], "interp_global", "Global SE(3) align (interp)"),
                       (axs[1], "interp_firstframe", "First-frame align (interp)")]:
        v = variants.get(key)
        if v is None: continue
        ax.plot(v["P_ref"][:, 0], v["P_ref"][:, 1], "b-", lw=1.5, label="reference")
        ax.plot(v["P_slam_a"][:, 0], v["P_slam_a"][:, 1], "r--", lw=1.5, label="SLAM aligned")
        ax.scatter([v["P_ref"][0, 0]], [v["P_ref"][0, 1]], c="lime", s=120,
                   marker="o", edgecolors="k", zorder=5)
        ax.scatter([v["P_ref"][-1, 0]], [v["P_ref"][-1, 1]], c="orange", s=120,
                   marker="*", edgecolors="k", zorder=5)
        ax.set_aspect("equal"); ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
        ax.set_title(f"{h}\nATE RMS = {v['ate_rms']:.3f} m"); ax.legend(); ax.grid(alpha=0.3)
    fig.savefig(out / "compare_traj_topdown_2way.png", dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved {out / 'compare_traj_topdown_2way.png'}")


if __name__ == "__main__":
    main()
