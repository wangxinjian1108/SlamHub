#!/usr/bin/env python3
"""Extract LIO-SAM trajectory from transformations.pcd → TUM format.

transformations.pcd schema (set by LIO-SAM's PointXYZIRPYT):
    FIELDS x y z intensity roll pitch yaw time
    SIZE   4 4 4 4         4    4     4   8
    TYPE   F F F F         F    F     F   F
where intensity = keyframe index (float32), time = absolute scan timestamp (double).
"""
import sys
from pathlib import Path
import numpy as np


def parse_pcd_xyzirpyt(pcd_path):
    with open(pcd_path, "rb") as f:
        while True:
            line = f.readline().decode("ascii", errors="ignore").strip()
            if line.startswith("DATA"):
                break
        dt = np.dtype([
            ("x", "f4"), ("y", "f4"), ("z", "f4"),
            ("intensity", "f4"),
            ("roll", "f4"), ("pitch", "f4"), ("yaw", "f4"),
            ("time", "f8"),
        ])
        raw = np.frombuffer(f.read(), dtype=dt)
    return raw


def rpy_to_quat(roll, pitch, yaw):
    cy = np.cos(yaw * 0.5);  sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5); sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5);  sr = np.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def main():
    pcd = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("output/liosam_run/transformations.pcd")
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else pcd.parent / "trajectory.txt"
    data = parse_pcd_xyzirpyt(pcd)
    qx, qy, qz, qw = rpy_to_quat(data["roll"], data["pitch"], data["yaw"])
    with open(out, "w") as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for i in range(len(data)):
            f.write(f"{data['time'][i]:.9f} {data['x'][i]:.6f} {data['y'][i]:.6f} {data['z'][i]:.6f} "
                    f"{qx[i]:.9f} {qy[i]:.9f} {qz[i]:.9f} {qw[i]:.9f}\n")
    print(f"Wrote {len(data)} poses to {out}")
    print(f"X range: [{data['x'].min():.2f}, {data['x'].max():.2f}] m")
    print(f"Y range: [{data['y'].min():.2f}, {data['y'].max():.2f}] m")
    print(f"Z range: [{data['z'].min():.2f}, {data['z'].max():.2f}] m")
    # Trajectory length (sum of consecutive pose distances)
    diffs = np.diff(np.column_stack([data["x"], data["y"], data["z"]]), axis=0)
    length = float(np.linalg.norm(diffs, axis=1).sum())
    print(f"Trajectory length: {length:.2f} m")


if __name__ == "__main__":
    main()
