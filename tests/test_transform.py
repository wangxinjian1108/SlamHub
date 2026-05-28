"""Tests for scripts/common/transform.py coordinate transform utilities."""

import sys
import os

import numpy as np
import pytest

# Add project root to path so we can import scripts.common
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from scripts.common.transform import (
    euler_to_matrix,
    matrix_to_euler,
    quaternion_to_matrix,
    matrix_to_quaternion,
    make_homogeneous,
    invert_transform,
)


def test_euler_to_matrix_identity():
    """euler(0, 0, 0) should produce the 3x3 identity matrix."""
    R = euler_to_matrix(0.0, 0.0, 0.0)
    np.testing.assert_allclose(R, np.eye(3), atol=1e-12)


def test_euler_roundtrip():
    """Converting euler -> matrix -> euler should recover the original angles."""
    angles = [
        (0.1, 0.2, 0.3),
        (-0.5, 0.4, 1.0),
        (np.pi / 4, -np.pi / 6, np.pi / 3),
    ]
    for roll, pitch, yaw in angles:
        R = euler_to_matrix(roll, pitch, yaw)
        r2, p2, y2 = matrix_to_euler(R)
        np.testing.assert_allclose(r2, roll, atol=1e-10, err_msg=f"roll mismatch for {(roll, pitch, yaw)}")
        np.testing.assert_allclose(p2, pitch, atol=1e-10, err_msg=f"pitch mismatch for {(roll, pitch, yaw)}")
        np.testing.assert_allclose(y2, yaw, atol=1e-10, err_msg=f"yaw mismatch for {(roll, pitch, yaw)}")


def test_quaternion_to_matrix_identity():
    """Identity quaternion (0, 0, 0, 1) should produce the 3x3 identity matrix."""
    R = quaternion_to_matrix(0.0, 0.0, 0.0, 1.0)
    np.testing.assert_allclose(R, np.eye(3), atol=1e-12)


def test_quaternion_roundtrip():
    """Converting quat -> matrix -> quat should recover the original quaternion (up to sign)."""
    quats = [
        (0.0, 0.0, 0.0, 1.0),
        (0.5, 0.5, 0.5, 0.5),
        (0.1, 0.2, 0.3, 0.9274),  # approximately unit
    ]
    for qx, qy, qz, qw in quats:
        # Normalize input for fair comparison
        n = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        qx_n, qy_n, qz_n, qw_n = qx / n, qy / n, qz / n, qw / n
        # Canonical form: qw >= 0
        if qw_n < 0:
            qx_n, qy_n, qz_n, qw_n = -qx_n, -qy_n, -qz_n, -qw_n

        R = quaternion_to_matrix(qx, qy, qz, qw)
        qx2, qy2, qz2, qw2 = matrix_to_quaternion(R)

        # Handle sign ambiguity: q and -q represent the same rotation
        q_orig = np.array([qx_n, qy_n, qz_n, qw_n])
        q_recovered = np.array([qx2, qy2, qz2, qw2])
        dot = np.dot(q_orig, q_recovered)
        if dot < 0:
            q_recovered = -q_recovered

        np.testing.assert_allclose(q_recovered, q_orig, atol=1e-6,
                                   err_msg=f"quaternion roundtrip failed for {(qx, qy, qz, qw)}")


def test_make_homogeneous():
    """make_homogeneous should produce a correct 4x4 matrix."""
    R = np.eye(3)
    t = np.array([1.0, 2.0, 3.0])
    T = make_homogeneous(R, t)

    assert T.shape == (4, 4)
    np.testing.assert_allclose(T[:3, :3], R, atol=1e-12)
    np.testing.assert_allclose(T[:3, 3], t, atol=1e-12)
    np.testing.assert_allclose(T[3, :], [0, 0, 0, 1], atol=1e-12)


def test_invert_transform():
    """T @ invert_transform(T) should equal the 4x4 identity."""
    R = euler_to_matrix(0.3, -0.2, 0.7)
    t = np.array([5.0, -3.0, 1.5])
    T = make_homogeneous(R, t)
    T_inv = invert_transform(T)

    result = T @ T_inv
    np.testing.assert_allclose(result, np.eye(4), atol=1e-10)
