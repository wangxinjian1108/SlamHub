"""Utilities for aggregating per-frame transforms into a calibrated extrinsic.

Provides outlier filtering and quaternion-based rotation averaging for
robust extrinsic calibration from frame-level registration results.
"""

import numpy as np

from scripts.common.transform import matrix_to_quaternion, quaternion_to_matrix


def filter_outliers(transforms: list, threshold: float = 3.0) -> list:
    """Remove transforms whose translation deviates from the median.

    A transform is considered an outlier if its translation distance from
    the median translation exceeds threshold * std of all distances.

    Args:
        transforms: List of 4x4 homogeneous transform matrices.
        threshold: Number of standard deviations for outlier cutoff.

    Returns:
        Filtered list of transforms with outliers removed.
    """
    if len(transforms) < 3:
        return list(transforms)

    translations = np.array([T[:3, 3] for T in transforms])
    median_t = np.median(translations, axis=0)

    distances = np.linalg.norm(translations - median_t, axis=1)
    mean_dist = np.mean(distances)
    std = np.std(distances)

    if std < 1e-10:
        return list(transforms)

    mask = np.abs(distances - mean_dist) <= threshold * std
    return [T for T, keep in zip(transforms, mask) if keep]


def aggregate_transforms(transforms: list) -> np.ndarray:
    """Compute mean transform from a list of 4x4 homogeneous matrices.

    Translation is computed as the component-wise median.
    Rotation is averaged in quaternion space: quaternions are sign-flipped
    for consistency (dot product with first quaternion >= 0), averaged,
    and normalized before converting back to a rotation matrix.

    Args:
        transforms: List of 4x4 homogeneous transform matrices.

    Returns:
        4x4 homogeneous transform representing the mean.
    """
    if not transforms:
        return np.eye(4)

    translations = np.array([T[:3, 3] for T in transforms])
    median_t = np.median(translations, axis=0)

    # Convert rotations to quaternions
    quats = []
    for T in transforms:
        R = T[:3, :3]
        qx, qy, qz, qw = matrix_to_quaternion(R)
        quats.append(np.array([qx, qy, qz, qw]))

    quats = np.array(quats)

    # Ensure consistent sign: flip if dot product with first quat < 0
    ref = quats[0]
    for i in range(1, len(quats)):
        if np.dot(quats[i], ref) < 0:
            quats[i] = -quats[i]

    # Average and normalize
    mean_quat = np.mean(quats, axis=0)
    mean_quat = mean_quat / np.linalg.norm(mean_quat)

    qx, qy, qz, qw = mean_quat
    R_mean = quaternion_to_matrix(qx, qy, qz, qw)

    T_mean = np.eye(4)
    T_mean[:3, :3] = R_mean
    T_mean[:3, 3] = median_t

    return T_mean
