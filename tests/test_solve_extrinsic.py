"""Tests for scripts/solve_extrinsic_utils.py extrinsic solver utilities."""

import sys
import os

import numpy as np
import pytest

# Add project root to path so we can import scripts.solve_extrinsic_utils
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from scripts.solve_extrinsic_utils import aggregate_transforms, filter_outliers


def test_aggregate_transforms_identity():
    """Aggregating 10 identity matrices should produce the identity transform."""
    transforms = [np.eye(4) for _ in range(10)]
    result = aggregate_transforms(transforms)

    assert result.shape == (4, 4)
    np.testing.assert_allclose(result, np.eye(4), atol=1e-10)


def test_aggregate_transforms_consistent_rotation():
    """Aggregating identical non-trivial transforms should return that transform."""
    T = np.eye(4)
    # Apply a small rotation around Z
    angle = 0.3
    T[0, 0] = np.cos(angle)
    T[0, 1] = -np.sin(angle)
    T[1, 0] = np.sin(angle)
    T[1, 1] = np.cos(angle)
    T[:3, 3] = [1.0, 2.0, 3.0]

    transforms = [T.copy() for _ in range(10)]
    result = aggregate_transforms(transforms)

    np.testing.assert_allclose(result, T, atol=1e-10)


def test_aggregate_transforms_empty():
    """Aggregating an empty list should return identity."""
    result = aggregate_transforms([])
    np.testing.assert_allclose(result, np.eye(4), atol=1e-10)


def test_filter_outliers():
    """10 identity transforms + 1 outlier at [100,100,100] should filter to 10."""
    transforms = [np.eye(4) for _ in range(10)]

    # Add an outlier with large translation
    outlier = np.eye(4)
    outlier[:3, 3] = [100.0, 100.0, 100.0]
    transforms.append(outlier)

    filtered = filter_outliers(transforms)

    assert len(filtered) == 10
    # Verify all remaining transforms are identity (no outlier)
    for T in filtered:
        np.testing.assert_allclose(T[:3, 3], [0.0, 0.0, 0.0], atol=1e-10)


def test_filter_outliers_few_transforms():
    """With fewer than 3 transforms, return as-is without filtering."""
    transforms = [np.eye(4), np.eye(4)]
    filtered = filter_outliers(transforms)
    assert len(filtered) == 2


def test_filter_outliers_no_outliers():
    """When all transforms are identical, none should be filtered."""
    transforms = []
    for _ in range(20):
        T = np.eye(4)
        T[:3, 3] = [1.0, 2.0, 3.0]
        transforms.append(T)

    filtered = filter_outliers(transforms)
    # All identical -> std < 1e-10 -> returns as-is
    assert len(filtered) == 20
