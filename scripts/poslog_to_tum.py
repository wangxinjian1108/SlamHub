#!/usr/bin/env python3
"""Convert FAST-LIO pos_log.txt -> TUM trajectory (+ optional covariance CSV).

FAST-LIO pos_log columns (vanilla 25, C1-patched 31):
    0: time = lidar_beg_time - first_lidar_time   (seconds, relative)
    1-3: rotation as so3 vector (axis*angle)
    4-6: position (x, y, z)
    7-9: omega (zeros)
    10-12: velocity
    13-15: zeros (acc placeholder)
    16-18: bg
    19-21: ba
    22-24: gravity
    --- C1 patch additions ---
    25-27: P(0,0), P(1,1), P(2,2)  pos variances (m^2)
    28-30: P(3,3), P(4,4), P(5,5)  rot variances (rad^2)

TUM line: timestamp tx ty tz qx qy qz qw

When the input has 31 columns, this script also emits a side CSV
`<trajectory_dir>/pose_covariance.csv` with absolute timestamp + 6
variance columns, for downstream weighting (see C1 in the eval report).
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
    n_cols = data.shape[1] if data.ndim == 2 else len(data)
    has_cov = n_cols >= 31

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

    if has_cov:
        cov_path = out.parent / "pose_covariance.csv"
        with open(cov_path, "w") as f:
            f.write("timestamp,var_x,var_y,var_z,var_rx,var_ry,var_rz\n")
            for row in data:
                ts = t0 + row[0]
                f.write(f"{ts:.9f},"
                        f"{row[25]:.6e},{row[26]:.6e},{row[27]:.6e},"
                        f"{row[28]:.6e},{row[29]:.6e},{row[30]:.6e}\n")
        print(f"Wrote SLAM covariance to {cov_path}")
    else:
        print(f"  (pos_log has {n_cols} cols, no C1 cov diag; covariance CSV skipped)")


if __name__ == "__main__":
    main()
