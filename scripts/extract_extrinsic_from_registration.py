#!/usr/bin/env python3
"""Compute calibrated extrinsic T_baselink_secondary from 04 registration output.

Frame_transforms.txt rows are T_world_secondary (refined by ICP).
Primary trajectory.txt gives T_world_baselink at each pose timestamp.
For each frame: T_baselink_secondary = T_world_baselink^-1 @ T_world_secondary.
Aggregate per-frame extrinsics with median translation + average quaternion.

Compare to the YAML initial guess and report delta.

Usage:
    python3 scripts/extract_extrinsic_from_registration.py \
        --primary-trajectory output/ghcr_run_v3/trajectory.txt \
        --registration-dir output/ghcr_run_v3/registration \
        --initial-guess <recording>/application.yaml \
        --output output/ghcr_run_v3/calibrated_extrinsics.yaml
"""
import argparse
import sys
from pathlib import Path
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).parent))
from common.transform import (
    euler_to_matrix, matrix_to_euler,
    matrix_to_quaternion, quaternion_to_matrix,
    invert_transform, make_homogeneous,
)
from common.io import read_trajectory_tum

LIDAR_FRAME = {
    "remote_front_left_pointcloud": "FRAME_LIDAR_REMOTE_FRONT_LEFT",
    "remote_front_right_pointcloud": "FRAME_LIDAR_REMOTE_FRONT_RIGHT",
    "flash_front_pointcloud": "FRAME_LIDAR_FLASH_FRONT",
    "flash_rear_pointcloud": "FRAME_LIDAR_FLASH_REAR",
}


def pose_to_T(pose):
    _, tx, ty, tz, qx, qy, qz, qw = pose
    R = quaternion_to_matrix(qx, qy, qz, qw)
    return make_homogeneous(R, np.array([tx, ty, tz]))


def load_frame_transforms(path):
    ts_list, T_list = [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            ts = int(parts[0])
            vals = [float(x) for x in parts[1:13]]
            T = np.eye(4)
            T[:3, :] = np.array(vals).reshape(3, 4)
            ts_list.append(ts)
            T_list.append(T)
    return np.array(ts_list), T_list


def load_yaml_extrinsic_by_frame(yaml_path, frame_name):
    with open(yaml_path) as f:
        cfg = yaml.safe_load(f)
    for cal in cfg["vehicle"]["calibration"]["sensor_calibration"]:
        if cal["source"] == frame_name:
            t = cal["transformation"]
            R = euler_to_matrix(t[3], t[4], t[5])
            return make_homogeneous(R, np.array([t[0], t[1], t[2]]))
    return None


def load_yaml_extrinsic(yaml_path, secondary_name):
    frame = LIDAR_FRAME.get(secondary_name)
    if not frame:
        return None
    return load_yaml_extrinsic_by_frame(yaml_path, frame)


def aggregate_unweighted(transforms):
    """Median translation, sign-consistent quaternion mean (no weights)."""
    if not transforms:
        return np.eye(4), np.zeros(3)
    translations = np.array([T[:3, 3] for T in transforms])
    median_t = np.median(translations, axis=0)
    std_t = np.std(translations, axis=0)

    qs = np.array([matrix_to_quaternion(T[:3, :3]) for T in transforms])
    for i in range(1, len(qs)):
        if np.dot(qs[i], qs[0]) < 0:
            qs[i] = -qs[i]
    mean_q = qs.mean(axis=0)
    mean_q = mean_q / np.linalg.norm(mean_q)

    R = quaternion_to_matrix(*mean_q)
    return make_homogeneous(R, median_t), std_t


def aggregate_weighted(transforms, weights):
    """Weighted translation mean + sign-consistent weighted quaternion mean.

    std_t reported is the weighted standard deviation of translations
    around the weighted mean.
    """
    if not transforms:
        return np.eye(4), np.zeros(3), 0.0
    w = np.asarray(weights, dtype=np.float64)
    if w.sum() <= 0:
        return aggregate_unweighted(transforms) + (0.0,)
    w_n = w / w.sum()

    translations = np.array([T[:3, 3] for T in transforms])
    mean_t = (translations * w_n[:, None]).sum(axis=0)
    diff = translations - mean_t
    var_t = (diff ** 2 * w_n[:, None]).sum(axis=0)
    std_t = np.sqrt(var_t)

    qs = np.array([matrix_to_quaternion(T[:3, :3]) for T in transforms])
    for i in range(1, len(qs)):
        if np.dot(qs[i], qs[0]) < 0:
            qs[i] = -qs[i]
    mean_q = (qs * w_n[:, None]).sum(axis=0)
    mean_q = mean_q / np.linalg.norm(mean_q)
    R = quaternion_to_matrix(*mean_q)
    n_eff = float((w.sum() ** 2) / (w ** 2).sum())
    return make_homogeneous(R, mean_t), std_t, n_eff


def load_frame_quality(quality_path):
    """Return dict ts_ns -> dict(fitness, rmse, n_inliers, n_src)."""
    out = {}
    if not quality_path.exists():
        return out
    with open(quality_path) as f:
        header = f.readline()  # noqa
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split(",")
            ts = int(parts[0])
            out[ts] = dict(
                fitness=float(parts[1]),
                rmse=float(parts[2]),
                n_inliers=int(parts[3]),
                n_src=int(parts[4]) if len(parts) > 4 else 0,
            )
    return out


def load_slam_pose_covariance(path):
    """Load C1 pose_covariance.csv -> dict ts_s_round_ns -> (var_t (3,), var_r (3,)).

    We round abs-ts seconds back to int ns for lookup against ICP frame ts.
    """
    out = {}
    if not path or not path.exists():
        return out
    with open(path) as f:
        f.readline()  # header
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split(",")
            ts_ns = int(round(float(parts[0]) * 1e9))
            var_t = np.array([float(parts[1]), float(parts[2]), float(parts[3])])
            var_r = np.array([float(parts[4]), float(parts[5]), float(parts[6])])
            out[ts_ns] = (var_t, var_r)
    return out


def load_frame_information(info_path):
    """Return dict ts_ns -> 6×6 info matrix (block order: ω, t)."""
    out = {}
    if not info_path.exists():
        return out
    with open(info_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 37:
                continue
            ts = int(parts[0])
            vals = np.array([float(x) for x in parts[1:37]]).reshape(6, 6)
            out[ts] = vals
    return out


def aggregate_info_weighted(transforms, info_matrices):
    """Per-frame translation info matrix gives a 3-vector weight = diag of
    the translation block of each frame's info matrix (Σ_t⁻¹). Per-axis
    weighted mean: μ_a = Σ_i w_{i,a} x_{i,a} / Σ_i w_{i,a}. Reports
    per-axis effective n_eff."""
    n = len(transforms)
    if n == 0:
        return np.eye(4), np.zeros(3), np.zeros(3)
    P = np.array([T[:3, 3] for T in transforms])  # (N, 3)
    W = np.array([np.diag(I[3:6, 3:6]) for I in info_matrices])  # (N, 3)
    W = np.clip(W, 1e-12, None)
    Wn = W / W.sum(axis=0, keepdims=True)
    mean_t = (P * Wn).sum(axis=0)
    var_t = ((P - mean_t) ** 2 * Wn).sum(axis=0)
    std_t = np.sqrt(var_t)
    n_eff = (W.sum(axis=0) ** 2) / (W ** 2).sum(axis=0)

    # Rotation: scalar weight = sum of translation-info trace (proxy for
    # frame quality). For full 6×6 Mahalanobis we'd need full LS on so(3),
    # which is overkill for a per-frame aggregation here.
    scalar_w = W.sum(axis=1)  # (N,)
    scalar_w = scalar_w / scalar_w.sum()
    qs = np.array([matrix_to_quaternion(T[:3, :3]) for T in transforms])
    for i in range(1, len(qs)):
        if np.dot(qs[i], qs[0]) < 0:
            qs[i] = -qs[i]
    mean_q = (qs * scalar_w[:, None]).sum(axis=0)
    mean_q = mean_q / np.linalg.norm(mean_q)
    R = quaternion_to_matrix(*mean_q)
    return make_homogeneous(R, mean_t), std_t, n_eff


# ---------- SE(3) Lie algebra utilities ----------

def _skew(v):
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])


def _vee(W):
    return np.array([W[2, 1], W[0, 2], W[1, 0]])


def se3_exp(xi):
    """SE(3) exponential map. xi = (omega, rho) ∈ R^6, returns 4×4 T.
    Block order: omega first 3 (rotation), rho last 3 (twist translation)."""
    omega = xi[:3]
    rho = xi[3:]
    theta = float(np.linalg.norm(omega))
    Wx = _skew(omega)
    if theta < 1e-9:
        R = np.eye(3) + Wx + 0.5 * Wx @ Wx
        V = np.eye(3) + 0.5 * Wx + (1.0 / 6.0) * Wx @ Wx
    else:
        s = np.sin(theta); c = np.cos(theta)
        R = np.eye(3) + (s / theta) * Wx + ((1 - c) / theta ** 2) * Wx @ Wx
        V = np.eye(3) + ((1 - c) / theta ** 2) * Wx + \
            ((theta - s) / theta ** 3) * Wx @ Wx
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = V @ rho
    return T


def se3_log(T):
    """SE(3) logarithm map. T (4×4) → xi = (omega, rho) ∈ R^6."""
    R = T[:3, :3]
    t = T[:3, 3]
    cos_th = (np.trace(R) - 1.0) / 2.0
    cos_th = np.clip(cos_th, -1.0, 1.0)
    theta = float(np.arccos(cos_th))
    if theta < 1e-9:
        omega = 0.5 * _vee(R - R.T)
        Wx = _skew(omega)
        V_inv = np.eye(3) - 0.5 * Wx + (1.0 / 12.0) * Wx @ Wx
    else:
        omega = (theta / (2.0 * np.sin(theta))) * _vee(R - R.T)
        Wx = _skew(omega)
        coef = (1.0 - theta * np.cos(theta / 2.0) / (2.0 * np.sin(theta / 2.0))) / theta ** 2
        V_inv = np.eye(3) - 0.5 * Wx + coef * Wx @ Wx
    rho = V_inv @ t
    return np.concatenate([omega, rho])


def _schur_translation_info(W, ridge=1e-9):
    """Marginalize out the rotation block of a 6×6 info matrix to get the
    effective 3×3 translation info. W is in (omega, t) ordering, so:
        Ω = [[I_rr, I_rt],
             [I_tr, I_tt]]
    I_t_marg = I_tt - I_tr @ inv(I_rr) @ I_rt."""
    I_rr = W[:3, :3] + ridge * np.eye(3)
    I_rt = W[:3, 3:]
    I_tr = W[3:, :3]
    I_tt = W[3:, 3:]
    return I_tt - I_tr @ np.linalg.solve(I_rr, I_rt)


def aggregate_mahalanobis_translation(transforms, info_matrices, ridge=1e-9,
                                       trim_quantile=0.0):
    """B2-MH: full 3×3 Mahalanobis weighted mean on translation only, using
    Schur-complement marginal translation info per frame.

    Avoids the rotation-vs-translation unit/scale mismatch of the full 6×6.

    Translations: solve (Σ I_t_i) t̄ = Σ I_t_i t_i.
    Rotation: weighted quaternion mean with scalar w_i = trace(I_t_i).

    Returns (T̄, std_t_samples, info_t_sum, n_kept). std_t_samples is the
    weighted empirical std around the mean (comparable to B1/B2).
    """
    n = len(transforms)
    if n == 0:
        return np.eye(4), np.zeros(3), np.zeros((3, 3)), 0

    I_t = np.array([_schur_translation_info(W, ridge) for W in info_matrices])
    # Symmetrize and ridge for numerical stability
    I_t = 0.5 * (I_t + I_t.transpose(0, 2, 1)) + ridge * np.eye(3)[None, :, :]

    traces = np.array([np.trace(M) for M in I_t])
    keep = np.ones(n, dtype=bool)
    if trim_quantile > 0.0:
        lo, hi = np.quantile(traces, [trim_quantile, 1 - trim_quantile])
        keep = (traces >= lo) & (traces <= hi)
    I_t = I_t[keep]
    transforms_k = [transforms[i] for i in range(n) if keep[i]]
    n_kept = int(keep.sum())

    P = np.array([T[:3, 3] for T in transforms_k])  # (N, 3)
    H = I_t.sum(axis=0)                              # (3, 3)
    rhs = np.einsum("ijk,ik->j", I_t, P)
    mean_t = np.linalg.solve(H, rhs)

    # Weighted empirical std around mean_t (comparable to B1/B2 std)
    w = traces[keep]
    w = np.clip(w, 1e-12, None); w = w / w.sum()
    diff = P - mean_t
    var_t = (diff ** 2 * w[:, None]).sum(axis=0)
    std_t = np.sqrt(var_t)

    # Rotation: weighted quaternion mean
    qs = np.array([matrix_to_quaternion(T[:3, :3]) for T in transforms_k])
    for i in range(1, len(qs)):
        if np.dot(qs[i], qs[0]) < 0:
            qs[i] = -qs[i]
    mean_q = (qs * w[:, None]).sum(axis=0)
    mean_q = mean_q / np.linalg.norm(mean_q)
    R = quaternion_to_matrix(*mean_q)
    return make_homogeneous(R, mean_t), std_t, H, n_kept


def aggregate_mahalanobis_se3(transforms, info_matrices, max_iter=20,
                               tol=1e-8, ridge=1e-9, normalize_per_frame=True,
                               trim_quantile=0.0):
    """Iteratively solve weighted Karcher mean on SE(3) with per-frame 6×6
    information matrices (full Mahalanobis):

        T̄ = argmin  Σ_i  ξ_iᵀ Ω_i ξ_i   where ξ_i = Log(T_i · T̄⁻¹)

    Newton-style updates: δ = (Σ Ω_i)⁻¹ Σ Ω_i ξ_i ; T̄ ← Exp(δ) · T̄.

    Args:
        normalize_per_frame: if True, rescale each Ω_i so that trace(Ω_i)=1
            and then weight each frame by an outer scalar w_i ∝ 1 (uniform
            over the kept frames). This avoids a few high-inlier frames
            dominating the mean.
        trim_quantile: drop the top `q` and bottom `q` of frames by trace(Ω_i)
            to make the mean robust to extreme outliers.

    Returns (T̄, std_samples, info_sum, n_iter, n_kept). std_samples is the
    empirical weighted std of per-frame translations around the converged
    mean — directly comparable to B1/B2 std numbers.
    """
    n = len(transforms)
    if n == 0:
        return np.eye(4), np.zeros(3), np.zeros((6, 6)), 0, 0

    Omegas = np.stack(info_matrices, axis=0)  # (N, 6, 6)
    Omegas = 0.5 * (Omegas + Omegas.transpose(0, 2, 1))

    # Trim by trace if requested
    traces = np.array([np.trace(W) for W in Omegas])
    keep = np.ones(n, dtype=bool)
    if trim_quantile > 0.0:
        lo, hi = np.quantile(traces, [trim_quantile, 1 - trim_quantile])
        keep = (traces >= lo) & (traces <= hi)
    Omegas = Omegas[keep]
    transforms_k = [transforms[i] for i in range(n) if keep[i]]
    n_kept = int(keep.sum())

    if normalize_per_frame:
        tr = np.array([np.trace(W) for W in Omegas])
        tr = np.clip(tr, 1e-12, None)
        Omegas = Omegas / tr[:, None, None]
    Omegas = Omegas + ridge * np.eye(6)[None, :, :]

    # Initialize with unweighted mean (median translation + averaged quat)
    T_bar, _ = aggregate_unweighted(list(transforms_k))

    n_iter = 0
    for it in range(max_iter):
        n_iter = it + 1
        T_bar_inv = invert_transform(T_bar)
        xis = np.array([se3_log(T_i @ T_bar_inv) for T_i in transforms_k])

        # Solve (Σ Ω_i) δ = Σ Ω_i ξ_i
        H = Omegas.sum(axis=0)
        b = np.einsum("ijk,ik->j", Omegas, xis)
        try:
            delta = np.linalg.solve(H, b)
        except np.linalg.LinAlgError:
            delta = np.linalg.pinv(H) @ b

        T_bar = se3_exp(delta) @ T_bar
        if np.linalg.norm(delta) < tol:
            break

    # Empirical weighted std of translations around T_bar — comparable to B1/B2.
    P = np.array([T[:3, 3] for T in transforms_k])
    w = np.array([np.trace(W[3:6, 3:6]) for W in Omegas])
    w = np.clip(w, 1e-12, None); w = w / w.sum()
    diff = P - T_bar[:3, 3]
    var_t = (diff ** 2 * w[:, None]).sum(axis=0)
    std_t = np.sqrt(var_t)

    H = Omegas.sum(axis=0)
    return T_bar, std_t, H, n_iter, n_kept


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--primary-trajectory", type=Path, required=True)
    p.add_argument("--registration-dir", type=Path, required=True)
    p.add_argument("--initial-guess", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--no-weighting", action="store_true",
                   help="Force median aggregation (no quality or info weights).")
    p.add_argument("--info-weighting", action="store_true",
                   help="B2: use the per-frame 6×6 info matrix translation block "
                        "for weighted aggregation (overrides --no-weighting and "
                        "fitness/rmse scalar weights).")
    p.add_argument("--mahalanobis", action="store_true",
                   help="B2-MH: full 6×6 Mahalanobis Karcher mean on SE(3). "
                        "Uses Lie-algebra Newton iteration with each frame's "
                        "complete 6×6 info matrix (rotation, translation, and "
                        "coupling terms). Overrides --info-weighting.")
    p.add_argument("--slam-cov", type=Path, default=None,
                   help="C1: path to pose_covariance.csv produced by the patched "
                        "FAST-LIO + poslog_to_tum.py. When provided, per-frame "
                        "scalar weights are divided by trace(Σ_SLAM_t) so frames "
                        "where the primary trajectory itself is uncertain "
                        "(sharp turns, degenerate scenes) get downweighted.")
    args = p.parse_args()

    poses = read_trajectory_tum(args.primary_trajectory)
    pose_ts = (poses[:, 0] * 1e9).astype(np.int64)
    pose_Ts = [pose_to_T(p) for p in poses]

    # C1: optional SLAM pose covariance per frame
    slam_cov = load_slam_pose_covariance(args.slam_cov) if args.slam_cov else {}
    slam_cov_ts = np.array(sorted(slam_cov.keys()), dtype=np.int64) \
        if slam_cov else None
    if slam_cov:
        print(f"Loaded SLAM pose cov: {len(slam_cov)} entries from {args.slam_cov}")

    # FAST-LIO trajectory is T_world_IMU, not T_world_baselink. To recover
    # T_baselink_secondary we compose T_baselink_IMU @ T_imu_secondary.
    T_baselink_imu = load_yaml_extrinsic_by_frame(args.initial_guess, "FRAME_GNSS_IMU")
    if T_baselink_imu is None:
        print("Warning: FRAME_GNSS_IMU not found; assuming IMU == baselink.")
        T_baselink_imu = np.eye(4)
    else:
        print(f"IMU offset (baselink frame): t={T_baselink_imu[:3,3]}")

    result = {"calibrated_extrinsics": {}}

    for reg_subdir in sorted(args.registration_dir.iterdir()):
        if not reg_subdir.is_dir():
            continue
        name = reg_subdir.name
        ft_path = reg_subdir / "frame_transforms.txt"
        if not ft_path.exists():
            continue

        print(f"\n=== {name} ===")
        ts_list, T_world_sec_list = load_frame_transforms(ft_path)
        quality = {} if args.no_weighting else load_frame_quality(reg_subdir / "frame_quality.csv")
        need_info = args.info_weighting or args.mahalanobis
        infos = load_frame_information(reg_subdir / "frame_information.csv") \
            if need_info else {}

        extrinsics = []
        weights = []
        info_list = []
        slam_var_t_list = []
        for ts_ns, T_ws in zip(ts_list, T_world_sec_list):
            idx = int(np.argmin(np.abs(pose_ts - ts_ns)))
            T_wi = pose_Ts[idx]
            T_is = invert_transform(T_wi) @ T_ws
            T_bs = T_baselink_imu @ T_is
            extrinsics.append(T_bs)
            q = quality.get(int(ts_ns))
            if q is not None:
                w = q["fitness"] * q["n_inliers"] / (q["rmse"] ** 2 + 1e-6)
                if slam_cov_ts is not None:
                    j = int(np.argmin(np.abs(slam_cov_ts - ts_ns)))
                    var_t, _ = slam_cov[int(slam_cov_ts[j])]
                    w = w / (var_t.sum() + 1e-9)
                weights.append(w)
            info_list.append(infos.get(int(ts_ns)))
            if slam_cov_ts is not None:
                j = int(np.argmin(np.abs(slam_cov_ts - ts_ns)))
                slam_var_t_list.append(slam_cov[int(slam_cov_ts[j])][0])

        T_unw, std_unw = aggregate_unweighted(extrinsics)
        # B2: keep only frames that have a valid info matrix; if at least
        # 10 such frames exist, do info-weighted aggregation on that subset.
        valid_pairs = [(T, I) for T, I in zip(extrinsics, info_list) if I is not None]
        if args.mahalanobis and len(valid_pairs) >= 10:
            ex_v = [p[0] for p in valid_pairs]
            in_v = [p[1] for p in valid_pairs]
            T_calib, std_w, H_t, n_kept = aggregate_mahalanobis_translation(
                ex_v, in_v, trim_quantile=0.05,
            )
            n_eff = float(n_kept)
            agg_method = (f"Mahalanobis on translation w/ Schur complement "
                          f"(B2-MH: 3×3 marginal info, {n_kept}/{len(extrinsics)} kept)")
        elif args.info_weighting and len(valid_pairs) >= 10:
            ex_v = [p[0] for p in valid_pairs]
            in_v = [p[1] for p in valid_pairs]
            T_calib, std_w, n_eff_axes = aggregate_info_weighted(ex_v, in_v)
            n_eff = float(n_eff_axes.mean())
            agg_method = (f"info-weighted (B2: 6×6 ICP info matrix diagonal, "
                          f"using {len(valid_pairs)}/{len(extrinsics)} frames)")
        elif quality and len(weights) == len(extrinsics):
            # B1: per-frame quality-weighted aggregation
            T_calib, std_w, n_eff = aggregate_weighted(extrinsics, weights)
            agg_method = "weighted (fitness * inliers / rmse^2)"
        else:
            # Baseline: median translation, sign-consistent quaternion mean
            T_calib, std_w = T_unw, std_unw
            n_eff = float(len(extrinsics))
            agg_method = "unweighted median"

        tx, ty, tz = T_calib[:3, 3]
        roll, pitch, yaw = matrix_to_euler(T_calib[:3, :3])

        T_init = load_yaml_extrinsic(args.initial_guess, name)
        info = {
            "method": agg_method,
            "n_frames_total": len(extrinsics),
            "n_effective_weighted": float(n_eff) if quality else float(len(extrinsics)),
            "translation_xyz_m": [float(tx), float(ty), float(tz)],
            "euler_rpy_rad": [float(roll), float(pitch), float(yaw)],
            "translation_std_m": [float(s) for s in std_w],
            "translation_std_unweighted_m": [float(s) for s in std_unw],
        }
        if T_init is not None:
            dt = T_calib[:3, 3] - T_init[:3, 3]
            ir, ip, iy = matrix_to_euler(T_init[:3, :3])
            info["initial_xyz_rpy"] = [float(T_init[0, 3]), float(T_init[1, 3]),
                                       float(T_init[2, 3]),
                                       float(ir), float(ip), float(iy)]
            info["delta_translation_m"] = [float(dt[0]), float(dt[1]), float(dt[2])]
            info["delta_translation_norm_m"] = float(np.linalg.norm(dt))
            print(f"  initial : t=[{T_init[0,3]:7.4f},{T_init[1,3]:7.4f},{T_init[2,3]:7.4f}] "
                  f"rpy=[{ir:+.4f},{ip:+.4f},{iy:+.4f}]")
        print(f"  calib   : t=[{tx:7.4f},{ty:7.4f},{tz:7.4f}] "
              f"rpy=[{roll:+.4f},{pitch:+.4f},{yaw:+.4f}]")
        print(f"  std_t weighted   : [{std_w[0]:.4f},{std_w[1]:.4f},{std_w[2]:.4f}] m"
              + (f"  (n_eff={n_eff:.1f}/{len(extrinsics)})" if quality else ""))
        print(f"  std_t unweighted : [{std_unw[0]:.4f},{std_unw[1]:.4f},{std_unw[2]:.4f}] m")
        if T_init is not None:
            print(f"  Δt      : [{dt[0]:+.4f},{dt[1]:+.4f},{dt[2]:+.4f}] m "
                  f"(|Δt|={np.linalg.norm(dt):.4f} m)")

        result["calibrated_extrinsics"][name] = info

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with open(args.output, "w") as f:
        yaml.dump(result, f, default_flow_style=None, sort_keys=False)
    print(f"\nWrote {args.output}")


if __name__ == "__main__":
    main()
