"""Coordinate transform utilities for SLAM pipelines.

Provides conversions between Euler angles (extrinsic XYZ), quaternions,
rotation matrices, and homogeneous transforms.
"""

import numpy as np


def euler_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Convert Euler angles (extrinsic XYZ) to a 3x3 rotation matrix.

    Convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    """
    cx, sx = np.cos(roll), np.sin(roll)
    cy, sy = np.cos(pitch), np.sin(pitch)
    cz, sz = np.cos(yaw), np.sin(yaw)

    # Rx
    Rx = np.array([
        [1, 0, 0],
        [0, cx, -sx],
        [0, sx, cx],
    ])
    # Ry
    Ry = np.array([
        [cy, 0, sy],
        [0, 1, 0],
        [-sy, 0, cy],
    ])
    # Rz
    Rz = np.array([
        [cz, -sz, 0],
        [sz, cz, 0],
        [0, 0, 1],
    ])

    return Rz @ Ry @ Rx


def matrix_to_euler(R: np.ndarray) -> tuple:
    """Convert a 3x3 rotation matrix to Euler angles (roll, pitch, yaw).

    Assumes extrinsic XYZ convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll).
    Returns (roll, pitch, yaw) in radians.
    """
    # Handle gimbal lock
    sy = -R[2, 0]
    if np.abs(sy) >= 1.0 - 1e-6:
        pitch = np.arcsin(np.clip(sy, -1.0, 1.0))
        # Gimbal lock: set yaw = 0 and solve for roll
        yaw = 0.0
        roll = np.arctan2(R[0, 1], R[0, 2]) if sy > 0 else np.arctan2(-R[0, 1], R[0, 2])
    else:
        pitch = np.arcsin(sy)
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])

    return (roll, pitch, yaw)


def quaternion_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Convert a unit quaternion to a 3x3 rotation matrix.

    Quaternion format: (qx, qy, qz, qw) where qw is the scalar part.
    """
    # Normalize
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n

    R = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
    ])

    return R


def matrix_to_quaternion(R: np.ndarray) -> tuple:
    """Convert a 3x3 rotation matrix to a unit quaternion (qx, qy, qz, qw).

    Uses Shepperd's method for numerical stability.
    """
    trace = R[0, 0] + R[1, 1] + R[2, 2]

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    # Ensure qw >= 0 for canonical form
    if qw < 0:
        qx, qy, qz, qw = -qx, -qy, -qz, -qw

    return (qx, qy, qz, qw)


def make_homogeneous(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Combine a 3x3 rotation matrix and 3-vector translation into a 4x4 homogeneous transform."""
    t = np.asarray(t).flatten()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    """Invert a 4x4 homogeneous transform.

    For a rigid transform [R|t; 0 1], the inverse is [R^T | -R^T @ t; 0 1].
    """
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv
