"""Tests for scripts/common/io.py — PCD, TUM trajectory, and YAML IO."""

import tempfile
import os

import numpy as np
import pytest

import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from scripts.common.io import (
    read_pcd,
    write_pcd,
    read_trajectory_tum,
    write_trajectory_tum,
    read_yaml,
    write_yaml,
)


class TestPCDRoundtrip:
    def test_pcd_roundtrip(self):
        """Write 100 random (N,3) points, read back, assert allclose."""
        rng = np.random.default_rng(42)
        points = rng.standard_normal((100, 3)).astype(np.float32)

        with tempfile.NamedTemporaryFile(suffix=".pcd", delete=False) as f:
            tmp_path = f.name

        try:
            write_pcd(tmp_path, points)
            loaded = read_pcd(tmp_path)
            assert loaded.shape == (100, 3)
            assert loaded.dtype == np.float32
            np.testing.assert_allclose(loaded, points, atol=1e-7)
        finally:
            os.unlink(tmp_path)

    def test_pcd_with_intensity(self):
        """Write/read (N,4) points with intensity column."""
        rng = np.random.default_rng(123)
        points = rng.standard_normal((50, 4)).astype(np.float32)

        with tempfile.NamedTemporaryFile(suffix=".pcd", delete=False) as f:
            tmp_path = f.name

        try:
            write_pcd(tmp_path, points)
            loaded = read_pcd(tmp_path)
            assert loaded.shape == (50, 4)
            assert loaded.dtype == np.float32
            np.testing.assert_allclose(loaded, points, atol=1e-7)
        finally:
            os.unlink(tmp_path)


class TestTrajectoryTUM:
    def test_trajectory_tum_roundtrip(self):
        """Write 3 poses, read back, assert allclose."""
        poses = np.array(
            [
                [1000.0, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0],
                [1001.0, 4.0, 5.0, 6.0, 0.1, 0.2, 0.3, 0.9],
                [1002.0, 7.0, 8.0, 9.0, 0.4, 0.5, 0.6, 0.7],
            ],
            dtype=np.float64,
        )

        with tempfile.NamedTemporaryFile(suffix=".txt", delete=False) as f:
            tmp_path = f.name

        try:
            write_trajectory_tum(tmp_path, poses)
            loaded = read_trajectory_tum(tmp_path)
            assert loaded.shape == (3, 8)
            assert loaded.dtype == np.float64
            np.testing.assert_allclose(loaded, poses, atol=1e-5)
        finally:
            os.unlink(tmp_path)


class TestYAML:
    def test_yaml_roundtrip(self):
        """Write dict, read back, assert equal."""
        data = {
            "name": "test_config",
            "version": 2,
            "params": {"voxel_size": 0.05, "max_range": 100.0},
            "tags": ["slam", "lidar"],
        }

        with tempfile.NamedTemporaryFile(suffix=".yaml", delete=False) as f:
            tmp_path = f.name

        try:
            write_yaml(tmp_path, data)
            loaded = read_yaml(tmp_path)
            assert loaded == data
        finally:
            os.unlink(tmp_path)
