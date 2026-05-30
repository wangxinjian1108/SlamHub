#!/usr/bin/env python3
"""Convert FAST-LIO pos_log.txt -> TUM trajectory.

FAST-LIO pos_log columns (26):
    0: time = lidar_beg_time - first_lidar_time   (seconds, relative)
    1-3: rotation as so3 vector (axis*angle)
    4-6: position (x, y, z)
    7-9: omega (zeros)
    10-12: velocity
    13-15: zeros (acc placeholder)
    16-18: bg
    19-21: ba
    22-24: gravity
    25: blank

TUM line: timestamp tx ty tz qx qy qz qw

Absolute time = first_lidar_time + relative_time.
The first lidar_beg_time equals the header stamp of the first PointCloud2 we
wrote to the bag, which is the PCD filename timestamp (ns).
"""
import sys
from pathlib import Path
import numpy as np


def so3_to_quat(rot_vec):
    """Rodrigues rotation vector -> quaternion (qx,qy,qz,qw)."""
    theta = np.linalg.norm(rot_vec)
    if theta < 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    axis = rot_vec / theta
    s = np.sin(theta / 2.0)
    return axis[0] * s, axis[1] * s, axis[2] * s, np.cos(theta / 2.0)


def main():
    if len(sys.argv) < 3:
        print("Usage: poslog_to_tum.py <pos_log.txt> <out.txt> <first_lidar_time_s>", file=sys.stderr)
        sys.exit(1)
    pos_log = Path(sys.argv[1])
    out = Path(sys.argv[2])
    t0 = float(sys.argv[3])

    data = np.loadtxt(pos_log)
    with open(out, "w") as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for row in data:
            rel_t = row[0]
            ts = t0 + rel_t
            rot = row[1:4]
            tx, ty, tz = row[4], row[5], row[6]
            qx, qy, qz, qw = so3_to_quat(rot)
            f.write(f"{ts:.9f} {tx:.6f} {ty:.6f} {tz:.6f} "
                    f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n")
    print(f"Wrote {len(data)} poses to {out}")
    print(f"Time range: {t0 + data[0,0]:.6f} .. {t0 + data[-1,0]:.6f}")


if __name__ == "__main__":
    main()
