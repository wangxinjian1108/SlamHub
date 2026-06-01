#!/usr/bin/env python3
"""Compare LIO-SAM trajectory + map against FAST-LIO reference."""
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_tum(path):
    """Skip comment lines, return (t, xyz) ."""
    data = np.loadtxt(path, comments="#")
    return data[:, 0], data[:, 1:4]


def load_pos_log(path):
    """FAST-LIO pos_log columns: col0=time, col4-6=pos(x,y,z)."""
    data = np.loadtxt(path)
    return data[:, 0], data[:, 4:7]


def load_pcd_xyz(path, max_points=400000):
    with open(path, "rb") as f:
        fields, sizes, types, n = [], [], [], 0
        while True:
            line = f.readline().decode("ascii", errors="ignore").strip()
            if line.startswith("FIELDS"): fields = line.split()[1:]
            elif line.startswith("SIZE"):  sizes = [int(x) for x in line.split()[1:]]
            elif line.startswith("TYPE"):  types = line.split()[1:]
            elif line.startswith("POINTS"): n = int(line.split()[1])
            elif line.startswith("DATA"):  break
        dm = {"F": "f", "I": "i", "U": "u"}
        dt = np.dtype([(nm, f"{dm[t]}{s}") for nm, t, s in zip(fields, types, sizes)])
        raw = np.frombuffer(f.read(n * dt.itemsize), dtype=dt, count=n)
    xyz = np.column_stack([raw["x"], raw["y"], raw["z"]]).astype(np.float32)
    xyz = xyz[~np.isnan(xyz).any(axis=1)]
    if len(xyz) > max_points:
        xyz = xyz[np.random.choice(len(xyz), max_points, replace=False)]
    return xyz


def plot_topdown(fl_xyz, ls_xyz, out_path):
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.plot(fl_xyz[:, 0], fl_xyz[:, 1], "b-", lw=1.5, label=f"FAST-LIO ({len(fl_xyz)} poses)")
    ax.plot(ls_xyz[:, 0], ls_xyz[:, 1], "r--", lw=1.5, label=f"LIO-SAM ({len(ls_xyz)} keyframes)")
    ax.scatter(fl_xyz[0, 0], fl_xyz[0, 1], c="green", s=80, marker="o", zorder=5, label="start")
    ax.scatter(fl_xyz[-1, 0], fl_xyz[-1, 1], c="blue", s=80, marker="x", zorder=5, label="FAST-LIO end")
    ax.scatter(ls_xyz[-1, 0], ls_xyz[-1, 1], c="red", s=80, marker="x", zorder=5, label="LIO-SAM end")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_title("Top-down trajectory comparison")
    ax.axis("equal"); ax.grid(True, alpha=0.3); ax.legend()
    fig.tight_layout(); fig.savefig(out_path, dpi=120); plt.close(fig)
    print(f"  wrote {out_path}")


def plot_xyz(fl_t, fl_xyz, ls_t, ls_xyz, out_path):
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    fl_t0 = fl_t - fl_t.min()
    ls_t0 = ls_t - ls_t.min()
    for i, lbl in enumerate("xyz"):
        axes[i].plot(fl_t0, fl_xyz[:, i], "b-", lw=1.2, label="FAST-LIO")
        axes[i].plot(ls_t0, ls_xyz[:, i], "r--", lw=1.2, label="LIO-SAM")
        axes[i].set_ylabel(f"{lbl} (m)"); axes[i].grid(True, alpha=0.3); axes[i].legend()
    axes[2].set_xlabel("time since start (s)")
    fig.suptitle("Per-axis trajectory: FAST-LIO vs LIO-SAM")
    fig.tight_layout(); fig.savefig(out_path, dpi=120); plt.close(fig)
    print(f"  wrote {out_path}")


def plot_map(ls_map_xyz, ls_xyz, out_path):
    fig, ax = plt.subplots(figsize=(10, 8))
    # downsample for top-down view, color by height
    ax.scatter(ls_map_xyz[:, 0], ls_map_xyz[:, 1], c=ls_map_xyz[:, 2],
               cmap="viridis", s=0.3, marker=".", alpha=0.5)
    ax.plot(ls_xyz[:, 0], ls_xyz[:, 1], "r-", lw=2.0, label="trajectory")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_title(f"LIO-SAM map ({len(ls_map_xyz)} pts, color = height)")
    ax.axis("equal"); ax.grid(True, alpha=0.3); ax.legend()
    fig.tight_layout(); fig.savefig(out_path, dpi=120); plt.close(fig)
    print(f"  wrote {out_path}")


def main():
    ls_dir = Path("output/liosam_run")
    fl_dir = Path("output/ghcr_run_v3")
    out_dir = ls_dir
    fl_t, fl_xyz = load_tum(fl_dir / "trajectory.txt")
    ls_t, ls_xyz = load_tum(ls_dir / "trajectory.txt")
    print(f"FAST-LIO: {len(fl_t)} poses, length="
          f"{np.linalg.norm(np.diff(fl_xyz, axis=0), axis=1).sum():.2f}m")
    print(f"LIO-SAM : {len(ls_t)} keyframes, length="
          f"{np.linalg.norm(np.diff(ls_xyz, axis=0), axis=1).sum():.2f}m")
    plot_topdown(fl_xyz, ls_xyz, out_dir / "compare_topdown.png")
    plot_xyz(fl_t, fl_xyz, ls_t, ls_xyz, out_dir / "compare_xyz.png")
    print("Reading LIO-SAM GlobalMap.pcd ...")
    ls_map = load_pcd_xyz(ls_dir / "GlobalMap.pcd", max_points=400_000)
    print(f"  {len(ls_map)} map points sampled")
    plot_map(ls_map, ls_xyz, out_dir / "liosam_map_with_trajectory.png")


if __name__ == "__main__":
    main()
