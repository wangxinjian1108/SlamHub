#!/usr/bin/env python3
"""Visualize FAST-LIO2 output: trajectory (from pos_log.txt) + map (scans.pcd)."""
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read_pos_log(path):
    """FAST-LIO pos_log: col0=time, col1-3=rot, col4-6=pos(x,y,z)."""
    data = np.loadtxt(path)
    t = data[:, 0]
    pos = data[:, 4:7]
    return t, pos


def read_pcd_xyz(path, max_points=400000):
    with open(path, "rb") as f:
        header, fields, sizes, types, n, fmt = [], [], [], [], 0, "binary"
        while True:
            line = f.readline().decode("ascii", errors="ignore").strip()
            header.append(line)
            if line.startswith("DATA"):
                fmt = line.split()[1]
                break
        for line in header:
            if line.startswith("FIELDS"): fields = line.split()[1:]
            elif line.startswith("SIZE"): sizes = [int(x) for x in line.split()[1:]]
            elif line.startswith("TYPE"): types = line.split()[1:]
            elif line.startswith("POINTS"): n = int(line.split()[1])
        dm = {"F": "f", "I": "i", "U": "u"}
        dt = np.dtype([(nm, f"{dm[t]}{s}") for nm, t, s in zip(fields, types, sizes)])
        raw = np.frombuffer(f.read(n * dt.itemsize), dtype=dt, count=n)
    xyz = np.column_stack([raw["x"], raw["y"], raw["z"]]).astype(np.float32)
    xyz = xyz[~np.isnan(xyz).any(axis=1)]
    if len(xyz) > max_points:
        idx = np.random.choice(len(xyz), max_points, replace=False)
        xyz = xyz[idx]
    return xyz


def main():
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("output/ghcr_run")
    t, pos = read_pos_log(out / "pos_log.txt")
    dist = np.sum(np.linalg.norm(np.diff(pos, axis=0), axis=1))
    print(f"Trajectory: {len(pos)} poses, length={dist:.1f}m, "
          f"x[{pos[:,0].min():.1f},{pos[:,0].max():.1f}] "
          f"y[{pos[:,1].min():.1f},{pos[:,1].max():.1f}] "
          f"z[{pos[:,2].min():.1f},{pos[:,2].max():.1f}]")

    # Trajectory top-down
    fig, ax = plt.subplots(figsize=(10, 10))
    sc = ax.scatter(pos[:, 0], pos[:, 1], c=t - t[0], cmap="viridis", s=8)
    ax.plot(pos[:, 0], pos[:, 1], "k-", lw=0.4, alpha=0.5)
    ax.scatter([pos[0,0]],[pos[0,1]], c="lime", s=120, marker="o", label="start", zorder=5, edgecolors="k")
    ax.scatter([pos[-1,0]],[pos[-1,1]], c="red", s=120, marker="*", label="end", zorder=5, edgecolors="k")
    ax.set_aspect("equal"); ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_title(f"FAST-LIO2 Trajectory (top-down) — {len(pos)} poses, {dist:.1f}m")
    ax.legend(); plt.colorbar(sc, label="time (s)")
    fig.savefig(out / "fastlio_trajectory.png", dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved {out/'fastlio_trajectory.png'}")

    # Trajectory XYZ vs time
    fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    for i, lbl in enumerate("XYZ"):
        axs[i].plot(t - t[0], pos[:, i], lw=1.0)
        axs[i].set_ylabel(f"{lbl} (m)"); axs[i].grid(alpha=0.3)
    axs[2].set_xlabel("time (s)")
    axs[0].set_title("FAST-LIO2 Position vs Time")
    fig.savefig(out / "fastlio_position_xyz.png", dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved {out/'fastlio_position_xyz.png'}")

    # Map
    print("Loading map (downsampled)...")
    xyz = read_pcd_xyz(out / "scans.pcd")
    print(f"Map: {len(xyz)} points (downsampled), "
          f"x[{xyz[:,0].min():.0f},{xyz[:,0].max():.0f}] "
          f"y[{xyz[:,1].min():.0f},{xyz[:,1].max():.0f}] "
          f"z[{xyz[:,2].min():.0f},{xyz[:,2].max():.0f}]")
    fig, ax = plt.subplots(figsize=(12, 12))
    sc = ax.scatter(xyz[:, 0], xyz[:, 1], c=xyz[:, 2], cmap="jet", s=0.4)
    ax.plot(pos[:, 0], pos[:, 1], "k-", lw=1.5, label="trajectory")
    ax.set_aspect("equal"); ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_title(f"FAST-LIO2 Map (top-down, color=height) — {len(xyz)} pts shown")
    ax.legend(); plt.colorbar(sc, label="Z (m)")
    fig.savefig(out / "fastlio_map.png", dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved {out/'fastlio_map.png'}")


if __name__ == "__main__":
    main()
