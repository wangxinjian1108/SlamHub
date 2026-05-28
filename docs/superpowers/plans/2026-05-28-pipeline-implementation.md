# SlamHub Pipeline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the full single-LiDAR SLAM → cross-LiDAR calibration pipeline with pluggable registration, visualization, and a one-command runner.

**Architecture:** Independent numbered scripts communicate via filesystem (PCD files, TUM trajectory text, YAML configs). A `common/` library provides shared IO and transform utilities. A `registration/` package defines an abstract interface with ICP as the default backend. Visualization scripts use Open3D with optional headless rendering.

**Tech Stack:** Python 3.8+, numpy, scipy, open3d, pyyaml, ROS1 Noetic (steps 01-02 only)

---

## File Map

| File | Responsibility |
|------|---------------|
| `scripts/common/__init__.py` | Package marker |
| `scripts/common/transform.py` | Quaternion/euler/matrix conversions |
| `scripts/common/io.py` | Read/write PCD, TUM trajectory, YAML |
| `scripts/registration/__init__.py` | Package marker + method discovery |
| `scripts/registration/base.py` | RegistrationResult dataclass + RegistrationBase ABC |
| `scripts/registration/icp.py` | ICP implementation using Open3D |
| `scripts/02_run_slam.py` | Launch FAST-LIO2 via roslaunch + rosbag play |
| `scripts/03_export_slam_results.py` | Convert SLAM output → standard formats |
| `scripts/04_register_secondary.py` | Register secondary LiDAR against primary map |
| `scripts/05_solve_extrinsic.py` | Aggregate registration → final extrinsics |
| `scripts/viz/show_trajectory.py` | 3D trajectory visualization |
| `scripts/viz/show_map.py` | Point cloud map visualization |
| `scripts/viz/show_registration.py` | Registration overlay visualization |
| `scripts/run_all.sh` | One-command pipeline runner |
| `tests/test_transform.py` | Tests for transform utilities |
| `tests/test_io.py` | Tests for IO utilities |
| `tests/test_registration.py` | Tests for registration interface + ICP |
| `tests/test_solve_extrinsic.py` | Tests for extrinsic solver |

---

## Task 1: common/transform.py — Coordinate Transform Utilities

**Files:**
- Create: `scripts/common/__init__.py`
- Create: `scripts/common/transform.py`
- Create: `tests/test_transform.py`

- [ ] **Step 1: Write failing tests for transform utilities**

```python
# tests/test_transform.py
import numpy as np
from scripts.common.transform import (
    euler_to_matrix,
    matrix_to_euler,
    quaternion_to_matrix,
    matrix_to_quaternion,
    make_homogeneous,
    invert_transform,
)


def test_euler_to_matrix_identity():
    R = euler_to_matrix(0, 0, 0)
    np.testing.assert_allclose(R, np.eye(3), atol=1e-10)


def test_euler_roundtrip():
    roll, pitch, yaw = 0.1, -0.2, 0.3
    R = euler_to_matrix(roll, pitch, yaw)
    r2, p2, y2 = matrix_to_euler(R)
    np.testing.assert_allclose([r2, p2, y2], [roll, pitch, yaw], atol=1e-10)


def test_quaternion_to_matrix_identity():
    R = quaternion_to_matrix(0, 0, 0, 1)  # qx, qy, qz, qw
    np.testing.assert_allclose(R, np.eye(3), atol=1e-10)


def test_quaternion_roundtrip():
    qx, qy, qz, qw = 0.1, 0.2, 0.3, 0.9
    norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
    R = quaternion_to_matrix(qx, qy, qz, qw)
    qx2, qy2, qz2, qw2 = matrix_to_quaternion(R)
    # Quaternion sign ambiguity
    if qw2 * qw < 0:
        qx2, qy2, qz2, qw2 = -qx2, -qy2, -qz2, -qw2
    np.testing.assert_allclose([qx2, qy2, qz2, qw2], [qx, qy, qz, qw], atol=1e-10)


def test_make_homogeneous():
    R = np.eye(3)
    t = np.array([1.0, 2.0, 3.0])
    T = make_homogeneous(R, t)
    assert T.shape == (4, 4)
    np.testing.assert_allclose(T[:3, :3], R)
    np.testing.assert_allclose(T[:3, 3], t)
    np.testing.assert_allclose(T[3, :], [0, 0, 0, 1])


def test_invert_transform():
    R = euler_to_matrix(0.1, 0.2, 0.3)
    t = np.array([1.0, 2.0, 3.0])
    T = make_homogeneous(R, t)
    T_inv = invert_transform(T)
    result = T @ T_inv
    np.testing.assert_allclose(result, np.eye(4), atol=1e-10)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /root/code/SlamHub && python -m pytest tests/test_transform.py -v`
Expected: FAIL with ModuleNotFoundError

- [ ] **Step 3: Implement transform.py**

```python
# scripts/common/__init__.py
```

```python
# scripts/common/transform.py
import numpy as np


def euler_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Euler angles (roll, pitch, yaw) to 3x3 rotation matrix. Extrinsic XYZ convention."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])

    return Rz @ Ry @ Rx


def matrix_to_euler(R: np.ndarray) -> tuple:
    """3x3 rotation matrix to (roll, pitch, yaw). Inverse of euler_to_matrix."""
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0]**2 + R[1, 0]**2))
    if np.abs(np.cos(pitch)) < 1e-10:
        roll = 0.0
        yaw = np.arctan2(R[0, 1], R[1, 1])
    else:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def quaternion_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Quaternion (qx, qy, qz, qw) to 3x3 rotation matrix."""
    R = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)],
    ])
    return R


def matrix_to_quaternion(R: np.ndarray) -> tuple:
    """3x3 rotation matrix to quaternion (qx, qy, qz, qw)."""
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
    return qx, qy, qz, qw


def make_homogeneous(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Combine 3x3 rotation and 3-vector translation into 4x4 homogeneous transform."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    """Invert a 4x4 homogeneous transform."""
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /root/code/SlamHub && python -m pytest tests/test_transform.py -v`
Expected: All 6 tests PASS

- [ ] **Step 5: Commit**

```bash
git add scripts/common/__init__.py scripts/common/transform.py tests/test_transform.py
git commit -m "feat: add coordinate transform utilities (euler/quat/matrix)"
```

---

## Task 2: common/io.py — PCD and Trajectory IO

**Files:**
- Create: `scripts/common/io.py`
- Create: `tests/test_io.py`

- [ ] **Step 1: Write failing tests for IO utilities**

```python
# tests/test_io.py
import tempfile
from pathlib import Path

import numpy as np
from scripts.common.io import (
    read_pcd,
    write_pcd,
    read_trajectory_tum,
    write_trajectory_tum,
    read_yaml,
    write_yaml,
)


def test_pcd_roundtrip():
    points = np.random.rand(100, 3).astype(np.float32)
    with tempfile.NamedTemporaryFile(suffix=".pcd", delete=False) as f:
        path = Path(f.name)
    write_pcd(path, points)
    loaded = read_pcd(path)
    np.testing.assert_allclose(loaded, points, atol=1e-6)
    path.unlink()


def test_pcd_with_intensity():
    points = np.random.rand(50, 4).astype(np.float32)  # x,y,z,intensity
    with tempfile.NamedTemporaryFile(suffix=".pcd", delete=False) as f:
        path = Path(f.name)
    write_pcd(path, points)
    loaded = read_pcd(path)
    np.testing.assert_allclose(loaded, points, atol=1e-6)
    path.unlink()


def test_trajectory_tum_roundtrip():
    # 3 poses: timestamp tx ty tz qx qy qz qw
    poses = np.array([
        [1000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        [1001.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        [1002.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    ])
    with tempfile.NamedTemporaryFile(suffix=".txt", delete=False, mode="w") as f:
        path = Path(f.name)
    write_trajectory_tum(path, poses)
    loaded = read_trajectory_tum(path)
    np.testing.assert_allclose(loaded, poses, atol=1e-10)
    path.unlink()


def test_yaml_roundtrip():
    data = {"lidar": "front_left", "transform": [1.0, 2.0, 3.0, 0.0, 0.0, 0.0]}
    with tempfile.NamedTemporaryFile(suffix=".yaml", delete=False, mode="w") as f:
        path = Path(f.name)
    write_yaml(path, data)
    loaded = read_yaml(path)
    assert loaded == data
    path.unlink()
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /root/code/SlamHub && python -m pytest tests/test_io.py -v`
Expected: FAIL with ImportError

- [ ] **Step 3: Implement io.py**

```python
# scripts/common/io.py
from pathlib import Path

import numpy as np
import yaml


def read_pcd(path: Path) -> np.ndarray:
    """Read a PCD file, return (N, 3) or (N, 4) float32 array depending on fields."""
    with open(path, "rb") as f:
        header_lines = []
        while True:
            line = f.readline().decode("ascii", errors="ignore").strip()
            header_lines.append(line)
            if line.startswith("DATA"):
                break

        fields = []
        sizes = []
        types = []
        num_points = 0
        data_format = "binary"

        for line in header_lines:
            if line.startswith("FIELDS"):
                fields = line.split()[1:]
            elif line.startswith("SIZE"):
                sizes = [int(x) for x in line.split()[1:]]
            elif line.startswith("TYPE"):
                types = line.split()[1:]
            elif line.startswith("POINTS"):
                num_points = int(line.split()[1])
            elif line.startswith("DATA"):
                data_format = line.split()[1]

        dtype_map = {"F": "f", "I": "i", "U": "u"}
        dt = np.dtype(
            [(name, f"{dtype_map[t]}{s}") for name, t, s in zip(fields, types, sizes)]
        )

        if data_format == "binary":
            raw = np.frombuffer(f.read(num_points * dt.itemsize), dtype=dt, count=num_points)
        else:
            raw = np.loadtxt(f, dtype=dt, max_rows=num_points)

    if "intensity" in fields:
        out = np.zeros((num_points, 4), dtype=np.float32)
        out[:, 0] = raw["x"]
        out[:, 1] = raw["y"]
        out[:, 2] = raw["z"]
        out[:, 3] = raw["intensity"]
    else:
        out = np.zeros((num_points, 3), dtype=np.float32)
        out[:, 0] = raw["x"]
        out[:, 1] = raw["y"]
        out[:, 2] = raw["z"]
    return out


def write_pcd(path: Path, points: np.ndarray):
    """Write (N,3) or (N,4) float32 array to binary PCD file."""
    n = len(points)
    cols = points.shape[1]
    if cols == 4:
        fields = "x y z intensity"
        sizes = "4 4 4 4"
        types = "F F F F"
        counts = "1 1 1 1"
    else:
        fields = "x y z"
        sizes = "4 4 4"
        types = "F F F"
        counts = "1 1 1"

    header = (
        f"# .PCD v0.7 - Point Cloud Data file format\n"
        f"VERSION 0.7\n"
        f"FIELDS {fields}\n"
        f"SIZE {sizes}\n"
        f"TYPE {types}\n"
        f"COUNT {counts}\n"
        f"WIDTH {n}\n"
        f"HEIGHT 1\n"
        f"VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        f"DATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(points.astype(np.float32).tobytes())


def read_trajectory_tum(path: Path) -> np.ndarray:
    """Read TUM trajectory file. Returns (N, 8) array: timestamp tx ty tz qx qy qz qw."""
    lines = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            lines.append([float(x) for x in line.split()])
    return np.array(lines)


def write_trajectory_tum(path: Path, poses: np.ndarray):
    """Write TUM trajectory. poses is (N, 8): timestamp tx ty tz qx qy qz qw."""
    with open(path, "w") as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for row in poses:
            f.write(" ".join(f"{v:.10f}" for v in row) + "\n")


def read_yaml(path: Path) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def write_yaml(path: Path, data: dict):
    with open(path, "w") as f:
        yaml.dump(data, f, default_flow_style=False, allow_unicode=True)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /root/code/SlamHub && python -m pytest tests/test_io.py -v`
Expected: All 4 tests PASS

- [ ] **Step 5: Commit**

```bash
git add scripts/common/io.py tests/test_io.py
git commit -m "feat: add PCD/trajectory/YAML IO utilities"
```

---

## Task 3: registration/ — Base Interface + ICP Implementation

**Files:**
- Create: `scripts/registration/__init__.py`
- Create: `scripts/registration/base.py`
- Create: `scripts/registration/icp.py`
- Create: `tests/test_registration.py`

- [ ] **Step 1: Write failing tests**

```python
# tests/test_registration.py
import numpy as np
from scripts.common.transform import euler_to_matrix, make_homogeneous
from scripts.registration import get_registration_method
from scripts.registration.base import RegistrationBase, RegistrationResult
from scripts.registration.icp import ICPRegistration


def test_registration_result_fields():
    r = RegistrationResult(
        transformation=np.eye(4),
        fitness=0.95,
        inlier_rmse=0.01,
        num_inliers=100,
    )
    assert r.fitness == 0.95
    assert r.transformation.shape == (4, 4)


def test_icp_is_registration_base():
    icp = ICPRegistration()
    assert isinstance(icp, RegistrationBase)


def test_get_registration_method():
    method = get_registration_method("icp")
    assert isinstance(method, RegistrationBase)


def test_icp_identity_alignment():
    """Two identical clouds should produce near-identity transform."""
    np.random.seed(42)
    source = np.random.rand(500, 3).astype(np.float64) * 10
    target = source.copy()
    icp = ICPRegistration(max_correspondence_distance=1.0)
    result = icp.register(source, target, initial_guess=np.eye(4))
    np.testing.assert_allclose(result.transformation, np.eye(4), atol=0.01)
    assert result.fitness > 0.9


def test_icp_known_transform():
    """Apply a known transform to source, ICP should recover it."""
    np.random.seed(42)
    target = np.random.rand(1000, 3).astype(np.float64) * 10
    R = euler_to_matrix(0.0, 0.0, 0.05)  # small yaw rotation
    t = np.array([0.1, 0.2, 0.0])
    T_true = make_homogeneous(R, t)
    source = (R @ target.T).T + t
    icp = ICPRegistration(max_correspondence_distance=2.0)
    result = icp.register(source, target, initial_guess=np.eye(4))
    T_inv = np.linalg.inv(result.transformation)
    np.testing.assert_allclose(T_inv[:3, 3], t, atol=0.1)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /root/code/SlamHub && python -m pytest tests/test_registration.py -v`
Expected: FAIL with ModuleNotFoundError

- [ ] **Step 3: Implement registration package**

```python
# scripts/registration/__init__.py
from scripts.registration.base import RegistrationBase, RegistrationResult

_METHODS = {}


def register_method(name):
    def decorator(cls):
        _METHODS[name] = cls
        return cls
    return decorator


def get_registration_method(name: str, **kwargs) -> RegistrationBase:
    if not _METHODS:
        import scripts.registration.icp  # noqa: F401 trigger registration
    if name not in _METHODS:
        available = ", ".join(_METHODS.keys())
        raise ValueError(f"Unknown method '{name}'. Available: {available}")
    return _METHODS[name](**kwargs)
```

```python
# scripts/registration/base.py
from dataclasses import dataclass

import numpy as np


@dataclass
class RegistrationResult:
    transformation: np.ndarray  # 4x4 homogeneous transform
    fitness: float              # Overlap ratio [0, 1]
    inlier_rmse: float          # RMS error of inlier correspondences
    num_inliers: int


class RegistrationBase:
    def __init__(self, **kwargs):
        pass

    def register(
        self,
        source: np.ndarray,
        target: np.ndarray,
        initial_guess: np.ndarray = None,
    ) -> RegistrationResult:
        raise NotImplementedError
```

```python
# scripts/registration/icp.py
import numpy as np
import open3d as o3d

from scripts.registration import register_method
from scripts.registration.base import RegistrationBase, RegistrationResult


@register_method("icp")
class ICPRegistration(RegistrationBase):
    def __init__(self, max_correspondence_distance: float = 1.0, max_iteration: int = 50, **kwargs):
        super().__init__(**kwargs)
        self.max_correspondence_distance = max_correspondence_distance
        self.max_iteration = max_iteration

    def register(
        self,
        source: np.ndarray,
        target: np.ndarray,
        initial_guess: np.ndarray = None,
    ) -> RegistrationResult:
        if initial_guess is None:
            initial_guess = np.eye(4)

        src_pcd = o3d.geometry.PointCloud()
        src_pcd.points = o3d.utility.Vector3dVector(source[:, :3].astype(np.float64))

        tgt_pcd = o3d.geometry.PointCloud()
        tgt_pcd.points = o3d.utility.Vector3dVector(target[:, :3].astype(np.float64))

        result = o3d.pipelines.registration.registration_icp(
            src_pcd,
            tgt_pcd,
            self.max_correspondence_distance,
            initial_guess.astype(np.float64),
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=self.max_iteration),
        )

        return RegistrationResult(
            transformation=np.asarray(result.transformation),
            fitness=result.fitness,
            inlier_rmse=result.inlier_rmse,
            num_inliers=len(result.correspondence_set),
        )
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /root/code/SlamHub && python -m pytest tests/test_registration.py -v`
Expected: All 5 tests PASS

- [ ] **Step 5: Commit**

```bash
git add scripts/registration/ tests/test_registration.py
git commit -m "feat: add registration interface with ICP backend"
```

---

## Task 4: 02_run_slam.py — FAST-LIO2 Launcher

**Files:**
- Create: `scripts/02_run_slam.py`

- [ ] **Step 1: Implement 02_run_slam.py**

```python
#!/usr/bin/env python3
"""
Launch FAST-LIO2 on a rosbag and collect outputs.

Designed to run inside the FAST_LIO Docker container where ROS Noetic is available.

Usage:
    python 02_run_slam.py input.bag --config config/fastlio_at128p.yaml --output-dir output/slam/
"""

import argparse
import subprocess
import sys
import time
from pathlib import Path


def run_slam(bag_path: Path, config_path: Path, output_dir: Path):
    output_dir.mkdir(parents=True, exist_ok=True)

    launch_cmd = [
        "roslaunch", "fast_lio", "mapping.launch",
        f"config_file:={config_path.resolve()}",
        f"bag_file:={bag_path.resolve()}",
    ]

    print(f"Launching FAST-LIO2...")
    print(f"  Bag: {bag_path}")
    print(f"  Config: {config_path}")
    print(f"  Output: {output_dir}")

    # Start roscore if not running
    roscore_proc = None
    try:
        subprocess.run(["rostopic", "list"], capture_output=True, timeout=2)
    except (subprocess.TimeoutExpired, FileNotFoundError):
        print("Starting roscore...")
        roscore_proc = subprocess.Popen(["roscore"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(2)

    try:
        # Launch FAST-LIO2
        slam_proc = subprocess.Popen(launch_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

        # Play bag
        time.sleep(3)  # wait for FAST-LIO2 to initialize
        play_cmd = ["rosbag", "play", str(bag_path.resolve()), "--clock"]
        print("Playing bag...")
        play_proc = subprocess.run(play_cmd, capture_output=True)

        # Wait for SLAM to finish processing
        time.sleep(5)
        slam_proc.terminate()
        slam_proc.wait(timeout=10)

    finally:
        if roscore_proc:
            roscore_proc.terminate()

    # Collect outputs - FAST-LIO2 saves to PCD_FILE_DIR
    pcd_save_dir = Path.home() / "catkin_ws" / "src" / "FAST_LIO" / "PCD"
    if pcd_save_dir.exists():
        import shutil
        for f in pcd_save_dir.iterdir():
            shutil.copy2(f, output_dir / f.name)
        print(f"Copied SLAM outputs to {output_dir}")
    else:
        print(f"Warning: FAST-LIO2 PCD output directory not found at {pcd_save_dir}")
        print("Check FAST-LIO2 config for pcd_save_dir parameter.")

    print("SLAM complete.")


def main():
    parser = argparse.ArgumentParser(description="Run FAST-LIO2 on a rosbag")
    parser.add_argument("bag_path", type=Path, help="Input rosbag file")
    parser.add_argument("--config", type=Path, required=True, help="FAST-LIO2 config YAML")
    parser.add_argument("--output-dir", type=Path, default=Path("output/slam"), help="Output directory")

    args = parser.parse_args()

    if not args.bag_path.exists():
        print(f"Error: bag file not found: {args.bag_path}")
        sys.exit(1)

    run_slam(args.bag_path, args.config, args.output_dir)


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Commit**

```bash
git add scripts/02_run_slam.py
git commit -m "feat: add FAST-LIO2 launcher script"
```

---

## Task 5: 03_export_slam_results.py — Export SLAM Outputs

**Files:**
- Create: `scripts/03_export_slam_results.py`

- [ ] **Step 1: Implement 03_export_slam_results.py**

```python
#!/usr/bin/env python3
"""
Export FAST-LIO2 raw outputs to standardized formats.

Reads FAST-LIO2's native output (scans.pcd, trajectory files) and produces:
- global_map.pcd: full accumulated point cloud map
- trajectory.txt: TUM format poses
- frames/: per-frame point clouds in global coordinate system

Usage:
    python 03_export_slam_results.py output/slam/ --format tum
"""

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from common.io import read_pcd, write_pcd, write_trajectory_tum, read_yaml
from common.transform import quaternion_to_matrix, make_homogeneous


def find_slam_outputs(slam_dir: Path) -> dict:
    """Locate FAST-LIO2 output files in the given directory."""
    outputs = {}

    # FAST-LIO2 saves accumulated map as scans.pcd or similar
    for candidate in ["scans.pcd", "GlobalMap.pcd", "map.pcd"]:
        p = slam_dir / candidate
        if p.exists():
            outputs["map"] = p
            break

    # Trajectory: FAST-LIO2 may save as pos_log.txt or trajectory.txt
    for candidate in ["pos_log.txt", "trajectory.txt", "path.txt"]:
        p = slam_dir / candidate
        if p.exists():
            outputs["trajectory"] = p
            break

    # Per-frame scans directory
    frames_dir = slam_dir / "scans"
    if frames_dir.exists():
        outputs["frames_dir"] = frames_dir

    return outputs


def parse_fastlio_trajectory(traj_path: Path) -> np.ndarray:
    """Parse FAST-LIO2 trajectory to TUM format (N, 8): timestamp tx ty tz qx qy qz qw."""
    poses = []
    with open(traj_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [float(x) for x in line.split()]
            if len(parts) >= 8:
                # Assume: timestamp tx ty tz qx qy qz qw
                poses.append(parts[:8])
            elif len(parts) == 7:
                # Assume: timestamp tx ty tz roll pitch yaw → convert to quat
                from common.transform import euler_to_matrix, matrix_to_quaternion
                ts, tx, ty, tz, roll, pitch, yaw = parts
                R = euler_to_matrix(roll, pitch, yaw)
                qx, qy, qz, qw = matrix_to_quaternion(R)
                poses.append([ts, tx, ty, tz, qx, qy, qz, qw])
    return np.array(poses)


def export(slam_dir: Path, output_dir: Path):
    output_dir.mkdir(parents=True, exist_ok=True)
    frames_out = output_dir / "frames"
    frames_out.mkdir(exist_ok=True)

    outputs = find_slam_outputs(slam_dir)

    if "map" in outputs:
        map_data = read_pcd(outputs["map"])
        write_pcd(output_dir / "global_map.pcd", map_data)
        print(f"Exported global_map.pcd ({len(map_data)} points)")
    else:
        print("Warning: no map file found in SLAM output")

    if "trajectory" in outputs:
        poses = parse_fastlio_trajectory(outputs["trajectory"])
        write_trajectory_tum(output_dir / "trajectory.txt", poses)
        print(f"Exported trajectory.txt ({len(poses)} poses)")
    else:
        print("Warning: no trajectory file found in SLAM output")
        return

    if "frames_dir" in outputs:
        frame_files = sorted(outputs["frames_dir"].glob("*.pcd"))
        print(f"Exporting {len(frame_files)} per-frame clouds to global frame...")
        for i, frame_file in enumerate(frame_files):
            cloud = read_pcd(frame_file)
            if i < len(poses):
                ts, tx, ty, tz, qx, qy, qz, qw = poses[i]
                R = quaternion_to_matrix(qx, qy, qz, qw)
                T = make_homogeneous(R, np.array([tx, ty, tz]))
                # Transform to global frame
                pts = cloud[:, :3]
                pts_h = np.hstack([pts, np.ones((len(pts), 1))])
                pts_global = (T @ pts_h.T).T[:, :3]
                if cloud.shape[1] == 4:
                    out = np.hstack([pts_global, cloud[:, 3:4]])
                else:
                    out = pts_global
                write_pcd(frames_out / f"{i:06d}.pcd", out.astype(np.float32))
            else:
                write_pcd(frames_out / f"{i:06d}.pcd", cloud)

            if (i + 1) % 100 == 0:
                print(f"  {i + 1}/{len(frame_files)} frames")
        print(f"Exported {len(frame_files)} frames")
    else:
        print("No per-frame scans directory found. Skipping frame export.")

    print("Export complete.")


def main():
    parser = argparse.ArgumentParser(description="Export SLAM results to standard formats")
    parser.add_argument("slam_dir", type=Path, help="SLAM output directory")
    parser.add_argument("--output-dir", type=Path, default=None, help="Export output directory (default: same as slam_dir)")
    parser.add_argument("--format", choices=["tum"], default="tum", help="Trajectory format")

    args = parser.parse_args()

    if not args.slam_dir.exists():
        print(f"Error: SLAM directory not found: {args.slam_dir}")
        sys.exit(1)

    output_dir = args.output_dir or args.slam_dir
    export(args.slam_dir, output_dir)


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Commit**

```bash
git add scripts/03_export_slam_results.py
git commit -m "feat: add SLAM results export script"
```

---

## Task 6: 04_register_secondary.py — Cross-LiDAR Registration

**Files:**
- Create: `scripts/04_register_secondary.py`

- [ ] **Step 1: Implement 04_register_secondary.py**

```python
#!/usr/bin/env python3
"""
Register secondary LiDAR point clouds against the primary SLAM map.

Supports two modes:
- global: accumulate secondary clouds, register against full map
- frame: register each frame against local submap

Usage:
    python 04_register_secondary.py \
        --primary-map output/slam/global_map.pcd \
        --trajectory output/slam/trajectory.txt \
        --secondary-dir /path/to/raw_pointclouds/flash_front_pointcloud/ \
        --initial-guess /path/to/application.yaml \
        --method icp \
        --output-dir output/registration/flash_front/
"""

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from common.io import read_pcd, read_trajectory_tum, read_yaml, write_yaml
from common.transform import euler_to_matrix, make_homogeneous, invert_transform
from registration import get_registration_method


LIDAR_FRAME_MAP = {
    "remote_front_left_pointcloud": "FRAME_LIDAR_REMOTE_FRONT_LEFT",
    "remote_front_right_pointcloud": "FRAME_LIDAR_REMOTE_FRONT_RIGHT",
    "flash_front_pointcloud": "FRAME_LIDAR_FLASH_FRONT",
    "flash_rear_pointcloud": "FRAME_LIDAR_FLASH_REAR",
}


def load_initial_guess(yaml_path: Path, lidar_name: str) -> np.ndarray:
    """Load initial extrinsic guess from application.yaml."""
    config = read_yaml(yaml_path)
    frame_name = LIDAR_FRAME_MAP.get(lidar_name)
    if not frame_name:
        return np.eye(4)

    calibrations = config["vehicle"]["calibration"]["sensor_calibration"]
    for cal in calibrations:
        if cal["source"] == frame_name:
            t = cal["transformation"]
            R = euler_to_matrix(t[3], t[4], t[5])
            return make_homogeneous(R, np.array([t[0], t[1], t[2]]))
    return np.eye(4)


def find_closest_pose_idx(timestamp_ns: int, poses: np.ndarray) -> int:
    """Find the closest pose by timestamp."""
    pose_timestamps = poses[:, 0] * 1e9  # TUM timestamps in seconds → ns
    idx = np.argmin(np.abs(pose_timestamps - timestamp_ns))
    return idx


def extract_submap(global_map: np.ndarray, center: np.ndarray, radius: float) -> np.ndarray:
    """Extract points within radius of center from global map."""
    dists = np.linalg.norm(global_map[:, :3] - center, axis=1)
    mask = dists < radius
    return global_map[mask]


def register_frame_mode(
    primary_map, trajectory, secondary_dir, initial_guess, method, output_dir, submap_radius
):
    """Register each secondary frame against local submap."""
    pcd_files = sorted(secondary_dir.glob("*.pcd"))
    registrator = get_registration_method(method, max_correspondence_distance=submap_radius * 0.1)

    transforms = []
    stats = {"total_frames": len(pcd_files), "successful": 0, "failed": 0, "fitness_values": []}

    output_dir.mkdir(parents=True, exist_ok=True)

    for i, pcd_file in enumerate(pcd_files):
        timestamp_ns = int(pcd_file.stem)
        pose_idx = find_closest_pose_idx(timestamp_ns, trajectory)
        pose = trajectory[pose_idx]
        tx, ty, tz = pose[1], pose[2], pose[3]

        submap = extract_submap(primary_map, np.array([tx, ty, tz]), submap_radius)
        if len(submap) < 100:
            transforms.append(None)
            stats["failed"] += 1
            continue

        source = read_pcd(pcd_file)
        valid_mask = ~np.isnan(source[:, 0])
        source = source[valid_mask]

        result = registrator.register(source[:, :3], submap[:, :3], initial_guess)

        transforms.append(result.transformation)
        stats["successful"] += 1
        stats["fitness_values"].append(result.fitness)

        if (i + 1) % 50 == 0:
            print(f"  Frame {i + 1}/{len(pcd_files)}, fitness={result.fitness:.4f}")

    # Save results
    with open(output_dir / "frame_transforms.txt", "w") as f:
        f.write("# frame_idx timestamp_ns t00 t01 t02 t03 t10 t11 t12 t13 t20 t21 t22 t23\n")
        for i, (pcd_file, T) in enumerate(zip(pcd_files, transforms)):
            if T is None:
                continue
            ts = int(pcd_file.stem)
            vals = " ".join(f"{v:.10f}" for v in T[:3, :].flatten())
            f.write(f"{i} {ts} {vals}\n")

    stats["mean_fitness"] = float(np.mean(stats["fitness_values"])) if stats["fitness_values"] else 0.0
    stats["std_fitness"] = float(np.std(stats["fitness_values"])) if stats["fitness_values"] else 0.0
    del stats["fitness_values"]
    write_yaml(output_dir / "summary.yaml", stats)

    print(f"Registration complete: {stats['successful']}/{stats['total_frames']} frames succeeded")
    print(f"Mean fitness: {stats['mean_fitness']:.4f} ± {stats['std_fitness']:.4f}")


def register_global_mode(
    primary_map, trajectory, secondary_dir, initial_guess, method, output_dir
):
    """Accumulate all secondary clouds into global frame, register against full map."""
    from common.transform import quaternion_to_matrix

    pcd_files = sorted(secondary_dir.glob("*.pcd"))
    registrator = get_registration_method(method, max_correspondence_distance=2.0)

    output_dir.mkdir(parents=True, exist_ok=True)

    # Accumulate secondary clouds using primary trajectory + initial guess
    accumulated = []
    for i, pcd_file in enumerate(pcd_files):
        timestamp_ns = int(pcd_file.stem)
        pose_idx = find_closest_pose_idx(timestamp_ns, trajectory)
        pose = trajectory[pose_idx]
        ts, tx, ty, tz, qx, qy, qz, qw = pose

        R_body = quaternion_to_matrix(qx, qy, qz, qw)
        T_body = make_homogeneous(R_body, np.array([tx, ty, tz]))
        T_lidar_global = T_body @ initial_guess

        source = read_pcd(pcd_file)
        valid_mask = ~np.isnan(source[:, 0])
        source = source[valid_mask]

        pts = source[:, :3]
        pts_h = np.hstack([pts, np.ones((len(pts), 1))])
        pts_global = (T_lidar_global @ pts_h.T).T[:, :3]
        accumulated.append(pts_global)

        if (i + 1) % 100 == 0:
            print(f"  Accumulated {i + 1}/{len(pcd_files)} frames")

    accumulated_cloud = np.vstack(accumulated).astype(np.float64)
    print(f"Accumulated cloud: {len(accumulated_cloud)} points")

    # Downsample for registration
    import open3d as o3d
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(accumulated_cloud)
    pcd_down = pcd.voxel_down_sample(voxel_size=0.2)
    accumulated_down = np.asarray(pcd_down.points)

    result = registrator.register(accumulated_down, primary_map[:, :3], np.eye(4))

    print(f"Global registration: fitness={result.fitness:.4f}, rmse={result.inlier_rmse:.4f}")

    stats = {
        "mode": "global",
        "total_frames": len(pcd_files),
        "accumulated_points": len(accumulated_cloud),
        "downsampled_points": len(accumulated_down),
        "fitness": float(result.fitness),
        "inlier_rmse": float(result.inlier_rmse),
    }
    write_yaml(output_dir / "summary.yaml", stats)

    # Save the global refinement transform
    with open(output_dir / "frame_transforms.txt", "w") as f:
        f.write("# Global registration result (single transform for all frames)\n")
        vals = " ".join(f"{v:.10f}" for v in result.transformation[:3, :].flatten())
        f.write(f"0 0 {vals}\n")


def main():
    parser = argparse.ArgumentParser(description="Register secondary LiDAR against primary map")
    parser.add_argument("--primary-map", type=Path, required=True, help="Primary global map PCD")
    parser.add_argument("--trajectory", type=Path, required=True, help="Primary trajectory (TUM)")
    parser.add_argument("--secondary-dir", type=Path, required=True, help="Secondary LiDAR PCD directory")
    parser.add_argument("--initial-guess", type=Path, help="application.yaml for initial extrinsic")
    parser.add_argument("--method", default="icp", help="Registration method")
    parser.add_argument("--mode", choices=["frame", "global"], default="frame", help="Registration mode")
    parser.add_argument("--submap-radius", type=float, default=50.0, help="Submap radius for frame mode (meters)")
    parser.add_argument("--output-dir", type=Path, required=True, help="Output directory")

    args = parser.parse_args()

    lidar_name = args.secondary_dir.name
    initial_guess = np.eye(4)
    if args.initial_guess and args.initial_guess.exists():
        initial_guess = load_initial_guess(args.initial_guess, lidar_name)
        print(f"Initial guess loaded for {lidar_name}")

    primary_map = read_pcd(args.primary_map)
    trajectory = read_trajectory_tum(args.trajectory)
    print(f"Primary map: {len(primary_map)} points, trajectory: {len(trajectory)} poses")

    if args.mode == "frame":
        register_frame_mode(
            primary_map, trajectory, args.secondary_dir,
            initial_guess, args.method, args.output_dir, args.submap_radius,
        )
    else:
        register_global_mode(
            primary_map, trajectory, args.secondary_dir,
            initial_guess, args.method, args.output_dir,
        )


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Commit**

```bash
git add scripts/04_register_secondary.py
git commit -m "feat: add cross-LiDAR registration script (frame + global mode)"
```

---

## Task 7: 05_solve_extrinsic.py — Extrinsic Solver

**Files:**
- Create: `scripts/05_solve_extrinsic.py`
- Create: `tests/test_solve_extrinsic.py`

- [ ] **Step 1: Write failing test**

```python
# tests/test_solve_extrinsic.py
import tempfile
from pathlib import Path

import numpy as np

from scripts.common.io import write_yaml
from scripts.solve_extrinsic_utils import aggregate_transforms, filter_outliers


def test_aggregate_transforms_identity():
    transforms = [np.eye(4) for _ in range(10)]
    mean_T = aggregate_transforms(transforms)
    np.testing.assert_allclose(mean_T, np.eye(4), atol=1e-6)


def test_filter_outliers():
    transforms = [np.eye(4) for _ in range(10)]
    # Add one outlier
    outlier = np.eye(4)
    outlier[:3, 3] = [100, 100, 100]
    transforms.append(outlier)
    filtered = filter_outliers(transforms, threshold=3.0)
    assert len(filtered) == 10
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /root/code/SlamHub && python -m pytest tests/test_solve_extrinsic.py -v`
Expected: FAIL with ImportError

- [ ] **Step 3: Implement 05_solve_extrinsic.py**

```python
#!/usr/bin/env python3
"""
Solve final extrinsics from registration results.

Aggregates per-frame transforms, filters outliers, computes robust average.

Usage:
    python 05_solve_extrinsic.py \
        --registration-dir output/registration/ \
        --primary-lidar remote_front_left_pointcloud \
        --output output/calibration/extrinsics.yaml
"""

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from common.io import write_yaml
from common.transform import matrix_to_euler, matrix_to_quaternion


def load_frame_transforms(transforms_path: Path) -> list:
    """Load per-frame transforms from frame_transforms.txt."""
    transforms = []
    with open(transforms_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [float(x) for x in line.split()]
            # Format: frame_idx timestamp_ns t00 t01 t02 t03 t10 t11 t12 t13 t20 t21 t22 t23
            if len(parts) >= 14:
                vals = parts[2:14]
                T = np.eye(4)
                T[0, :] = vals[0:4]
                T[1, :] = vals[4:8]
                T[2, :] = vals[8:12]
                transforms.append(T)
    return transforms


def aggregate_transforms(transforms: list) -> np.ndarray:
    """Compute mean transform from a list of 4x4 transforms."""
    if not transforms:
        return np.eye(4)

    # Average translations
    translations = np.array([T[:3, 3] for T in transforms])
    mean_t = np.median(translations, axis=0)

    # Average rotations via quaternion averaging
    from common.transform import matrix_to_quaternion, quaternion_to_matrix
    quats = []
    for T in transforms:
        q = matrix_to_quaternion(T[:3, :3])
        quats.append(q)
    quats = np.array(quats)

    # Ensure consistent quaternion sign
    for i in range(1, len(quats)):
        if np.dot(quats[i], quats[0]) < 0:
            quats[i] = -quats[i]

    mean_q = np.mean(quats, axis=0)
    mean_q = mean_q / np.linalg.norm(mean_q)

    mean_R = quaternion_to_matrix(*mean_q)
    T_mean = np.eye(4)
    T_mean[:3, :3] = mean_R
    T_mean[:3, 3] = mean_t
    return T_mean


def filter_outliers(transforms: list, threshold: float = 3.0) -> list:
    """Remove transforms whose translation is more than threshold*std from median."""
    if len(transforms) < 3:
        return transforms

    translations = np.array([T[:3, 3] for T in transforms])
    median = np.median(translations, axis=0)
    dists = np.linalg.norm(translations - median, axis=1)
    std = np.std(dists)

    if std < 1e-10:
        return transforms

    mask = dists < threshold * std
    return [T for T, m in zip(transforms, mask) if m]


def solve(registration_dir: Path, primary_lidar: str, output_path: Path):
    output_path.parent.mkdir(parents=True, exist_ok=True)

    extrinsics = {"primary_lidar": primary_lidar, "secondary_lidars": {}}

    for lidar_dir in sorted(registration_dir.iterdir()):
        if not lidar_dir.is_dir():
            continue
        lidar_name = lidar_dir.name

        transforms_file = lidar_dir / "frame_transforms.txt"
        if not transforms_file.exists():
            print(f"Skipping {lidar_name}: no frame_transforms.txt")
            continue

        transforms = load_frame_transforms(transforms_file)
        print(f"{lidar_name}: loaded {len(transforms)} transforms")

        if not transforms:
            continue

        filtered = filter_outliers(transforms)
        print(f"  After outlier filtering: {len(filtered)} transforms")

        T_mean = aggregate_transforms(filtered)
        roll, pitch, yaw = matrix_to_euler(T_mean[:3, :3])
        tx, ty, tz = T_mean[:3, 3]

        extrinsics["secondary_lidars"][lidar_name] = {
            "transform_xyzrpy": [float(tx), float(ty), float(tz), float(roll), float(pitch), float(yaw)],
            "transform_matrix": T_mean.tolist(),
            "num_frames_used": len(filtered),
            "num_frames_total": len(transforms),
            "translation_std": float(np.std([T[:3, 3] for T in filtered], axis=0).mean()),
        }

        print(f"  Result: t=[{tx:.4f}, {ty:.4f}, {tz:.4f}] rpy=[{roll:.4f}, {pitch:.4f}, {yaw:.4f}]")

    write_yaml(output_path, extrinsics)
    print(f"\nExtrinsics saved to {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Solve extrinsics from registration results")
    parser.add_argument("--registration-dir", type=Path, required=True)
    parser.add_argument("--primary-lidar", type=str, required=True)
    parser.add_argument("--output", type=Path, required=True)

    args = parser.parse_args()
    solve(args.registration_dir, args.primary_lidar, args.output)


if __name__ == "__main__":
    main()
```

Also expose the utility functions for testing by placing them in a separate module:

```python
# scripts/solve_extrinsic_utils.py
"""Utility functions for extrinsic solving, importable for testing."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from common.transform import matrix_to_quaternion, quaternion_to_matrix


def aggregate_transforms(transforms: list) -> np.ndarray:
    """Compute mean transform from a list of 4x4 transforms."""
    if not transforms:
        return np.eye(4)
    translations = np.array([T[:3, 3] for T in transforms])
    mean_t = np.median(translations, axis=0)
    quats = []
    for T in transforms:
        q = matrix_to_quaternion(T[:3, :3])
        quats.append(q)
    quats = np.array(quats)
    for i in range(1, len(quats)):
        if np.dot(quats[i], quats[0]) < 0:
            quats[i] = -quats[i]
    mean_q = np.mean(quats, axis=0)
    mean_q = mean_q / np.linalg.norm(mean_q)
    mean_R = quaternion_to_matrix(*mean_q)
    T_mean = np.eye(4)
    T_mean[:3, :3] = mean_R
    T_mean[:3, 3] = mean_t
    return T_mean


def filter_outliers(transforms: list, threshold: float = 3.0) -> list:
    """Remove transforms whose translation is more than threshold*std from median."""
    if len(transforms) < 3:
        return transforms
    translations = np.array([T[:3, 3] for T in transforms])
    median = np.median(translations, axis=0)
    dists = np.linalg.norm(translations - median, axis=1)
    std = np.std(dists)
    if std < 1e-10:
        return transforms
    mask = dists < threshold * std
    return [T for T, m in zip(transforms, mask) if m]
```

Note: `05_solve_extrinsic.py` imports from `solve_extrinsic_utils.py` to avoid duplication:

```python
# In 05_solve_extrinsic.py, replace inline definitions with:
from solve_extrinsic_utils import aggregate_transforms, filter_outliers
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /root/code/SlamHub && python -m pytest tests/test_solve_extrinsic.py -v`
Expected: All 2 tests PASS

- [ ] **Step 5: Commit**

```bash
git add scripts/05_solve_extrinsic.py scripts/solve_extrinsic_utils.py tests/test_solve_extrinsic.py
git commit -m "feat: add extrinsic solver with outlier filtering"
```

---

## Task 8: viz/ — Visualization Scripts

**Files:**
- Create: `scripts/viz/show_trajectory.py`
- Create: `scripts/viz/show_map.py`
- Create: `scripts/viz/show_registration.py`

- [ ] **Step 1: Implement show_trajectory.py**

```python
#!/usr/bin/env python3
"""
Visualize 3D trajectory with path and orientation axes.

Usage:
    python viz/show_trajectory.py trajectory.txt [--save traj.png]
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import open3d as o3d

sys.path.insert(0, str(Path(__file__).parent.parent))
from common.io import read_trajectory_tum
from common.transform import quaternion_to_matrix


def create_trajectory_lineset(poses: np.ndarray) -> o3d.geometry.LineSet:
    points = poses[:, 1:4]
    lines = [[i, i + 1] for i in range(len(points) - 1)]

    # Color by time (blue → red)
    t_norm = np.linspace(0, 1, len(lines))
    colors = [[t, 0, 1 - t] for t in t_norm]

    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(points)
    ls.lines = o3d.utility.Vector2iVector(lines)
    ls.colors = o3d.utility.Vector3dVector(colors)
    return ls


def create_axes_at_poses(poses: np.ndarray, every_n: int = 20, length: float = 0.5) -> list:
    geometries = []
    for i in range(0, len(poses), every_n):
        ts, tx, ty, tz, qx, qy, qz, qw = poses[i]
        R = quaternion_to_matrix(qx, qy, qz, qw)
        origin = np.array([tx, ty, tz])

        mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=length)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = origin
        mesh.transform(T)
        geometries.append(mesh)
    return geometries


def main():
    parser = argparse.ArgumentParser(description="Visualize 3D trajectory")
    parser.add_argument("trajectory", type=Path, help="TUM trajectory file")
    parser.add_argument("--save", type=Path, default=None, help="Save screenshot to file (headless)")
    parser.add_argument("--axes-every", type=int, default=20, help="Show axes every N poses")

    args = parser.parse_args()
    poses = read_trajectory_tum(args.trajectory)
    print(f"Loaded {len(poses)} poses")

    lineset = create_trajectory_lineset(poses)
    axes = create_axes_at_poses(poses, every_n=args.axes_every)
    geometries = [lineset] + axes

    if args.save:
        vis = o3d.visualization.Visualizer()
        vis.create_window(visible=False, width=1920, height=1080)
        for g in geometries:
            vis.add_geometry(g)
        vis.get_view_control().set_zoom(0.5)
        vis.poll_events()
        vis.update_renderer()
        vis.capture_screen_image(str(args.save))
        vis.destroy_window()
        print(f"Saved to {args.save}")
    else:
        o3d.visualization.draw_geometries(geometries, window_name="Trajectory")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Implement show_map.py**

```python
#!/usr/bin/env python3
"""
Visualize point cloud map with configurable coloring.

Usage:
    python viz/show_map.py global_map.pcd [--color-by height|intensity] [--save map.png]
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import open3d as o3d

sys.path.insert(0, str(Path(__file__).parent.parent))
from common.io import read_pcd


def colorize_by_height(points: np.ndarray) -> np.ndarray:
    z = points[:, 2]
    z_min, z_max = z.min(), z.max()
    if z_max - z_min < 1e-6:
        return np.ones((len(points), 3)) * 0.5
    z_norm = (z - z_min) / (z_max - z_min)
    colors = np.zeros((len(points), 3))
    colors[:, 0] = z_norm       # red increases with height
    colors[:, 2] = 1 - z_norm   # blue decreases with height
    return colors


def colorize_by_intensity(points: np.ndarray) -> np.ndarray:
    if points.shape[1] < 4:
        return np.ones((len(points), 3)) * 0.5
    intensity = points[:, 3]
    i_max = intensity.max()
    if i_max < 1e-6:
        return np.ones((len(points), 3)) * 0.5
    i_norm = intensity / i_max
    colors = np.column_stack([i_norm, i_norm, i_norm])
    return colors


def main():
    parser = argparse.ArgumentParser(description="Visualize point cloud map")
    parser.add_argument("map_pcd", type=Path, help="PCD map file")
    parser.add_argument("--color-by", choices=["height", "intensity"], default="height")
    parser.add_argument("--save", type=Path, default=None, help="Save screenshot (headless)")
    parser.add_argument("--voxel-size", type=float, default=0.0, help="Downsample voxel size (0=no downsample)")

    args = parser.parse_args()
    points = read_pcd(args.map_pcd)
    print(f"Loaded {len(points)} points")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3])

    if args.color_by == "height":
        pcd.colors = o3d.utility.Vector3dVector(colorize_by_height(points))
    else:
        pcd.colors = o3d.utility.Vector3dVector(colorize_by_intensity(points))

    if args.voxel_size > 0:
        pcd = pcd.voxel_down_sample(args.voxel_size)
        print(f"Downsampled to {len(pcd.points)} points")

    if args.save:
        vis = o3d.visualization.Visualizer()
        vis.create_window(visible=False, width=1920, height=1080)
        vis.add_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()
        vis.capture_screen_image(str(args.save))
        vis.destroy_window()
        print(f"Saved to {args.save}")
    else:
        o3d.visualization.draw_geometries([pcd], window_name="Point Cloud Map")


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: Implement show_registration.py**

```python
#!/usr/bin/env python3
"""
Visualize registration overlay: primary map (red) + secondary cloud (blue).

Usage:
    python viz/show_registration.py global_map.pcd \
        --secondary output/registration/flash_front/ \
        --frame 100 \
        [--save reg.png]
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import open3d as o3d

sys.path.insert(0, str(Path(__file__).parent.parent))
from common.io import read_pcd, read_trajectory_tum


def main():
    parser = argparse.ArgumentParser(description="Visualize registration overlay")
    parser.add_argument("primary_map", type=Path, help="Primary map PCD")
    parser.add_argument("--secondary", type=Path, required=True, help="Registration output directory")
    parser.add_argument("--secondary-pcd-dir", type=Path, default=None, help="Secondary raw PCD directory")
    parser.add_argument("--frame", type=int, default=0, help="Frame index to visualize")
    parser.add_argument("--save", type=Path, default=None, help="Save screenshot (headless)")
    parser.add_argument("--voxel-size", type=float, default=0.1, help="Downsample voxel size for primary map")

    args = parser.parse_args()

    # Load primary map
    primary_points = read_pcd(args.primary_map)
    primary_pcd = o3d.geometry.PointCloud()
    primary_pcd.points = o3d.utility.Vector3dVector(primary_points[:, :3])
    primary_pcd.paint_uniform_color([1.0, 0.3, 0.3])  # red

    if args.voxel_size > 0:
        primary_pcd = primary_pcd.voxel_down_sample(args.voxel_size)

    # Load secondary frame if available
    geometries = [primary_pcd]

    if args.secondary_pcd_dir and args.secondary_pcd_dir.exists():
        pcd_files = sorted(args.secondary_pcd_dir.glob("*.pcd"))
        if args.frame < len(pcd_files):
            sec_points = read_pcd(pcd_files[args.frame])
            valid = sec_points[~np.isnan(sec_points[:, 0])]

            # Load transform if available
            transforms_file = args.secondary / "frame_transforms.txt"
            T = np.eye(4)
            if transforms_file.exists():
                with open(transforms_file) as f:
                    for line in f:
                        if line.startswith("#"):
                            continue
                        parts = line.strip().split()
                        if int(parts[0]) == args.frame:
                            vals = [float(x) for x in parts[2:14]]
                            T[0, :] = vals[0:4]
                            T[1, :] = vals[4:8]
                            T[2, :] = vals[8:12]
                            break

            sec_pcd = o3d.geometry.PointCloud()
            pts_h = np.hstack([valid[:, :3], np.ones((len(valid), 1))])
            pts_transformed = (T @ pts_h.T).T[:, :3]
            sec_pcd.points = o3d.utility.Vector3dVector(pts_transformed)
            sec_pcd.paint_uniform_color([0.3, 0.3, 1.0])  # blue
            geometries.append(sec_pcd)
            print(f"Secondary frame {args.frame}: {len(valid)} points (blue)")

    print(f"Primary map: {len(primary_pcd.points)} points (red)")

    if args.save:
        vis = o3d.visualization.Visualizer()
        vis.create_window(visible=False, width=1920, height=1080)
        for g in geometries:
            vis.add_geometry(g)
        vis.poll_events()
        vis.update_renderer()
        vis.capture_screen_image(str(args.save))
        vis.destroy_window()
        print(f"Saved to {args.save}")
    else:
        o3d.visualization.draw_geometries(geometries, window_name="Registration Overlay")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Commit**

```bash
git add scripts/viz/
git commit -m "feat: add visualization scripts (trajectory, map, registration)"
```

---

## Task 9: run_all.sh — Pipeline Runner

**Files:**
- Create: `scripts/run_all.sh`

- [ ] **Step 1: Implement run_all.sh**

```bash
#!/bin/bash
set -euo pipefail

# SlamHub Pipeline Runner
# Usage: ./run_all.sh <recording_dir> [primary_lidar] [output_dir]

RECORDING_DIR="${1:?Usage: $0 <recording_dir> [primary_lidar] [output_dir]}"
PRIMARY_LIDAR="${2:-remote_front_left_pointcloud}"
OUTPUT_DIR="${3:-output/$(basename "$RECORDING_DIR")}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=== SlamHub Pipeline ==="
echo "Recording: $RECORDING_DIR"
echo "Primary LiDAR: $PRIMARY_LIDAR"
echo "Output: $OUTPUT_DIR"
echo ""

# Step 1: Convert to rosbag
echo ">>> Step 1: Converting PCD+IMU to rosbag..."
mkdir -p "$OUTPUT_DIR/bags"
python "$SCRIPT_DIR/01_convert_to_rosbag.py" \
    "$RECORDING_DIR" \
    --lidar "$PRIMARY_LIDAR" \
    -o "$OUTPUT_DIR/bags/${PRIMARY_LIDAR}.bag"
echo ""

# Step 2: Run SLAM
echo ">>> Step 2: Running FAST-LIO2..."
python "$SCRIPT_DIR/02_run_slam.py" \
    "$OUTPUT_DIR/bags/${PRIMARY_LIDAR}.bag" \
    --config "$SCRIPT_DIR/../config/fastlio_at128p.yaml" \
    --output-dir "$OUTPUT_DIR/slam/"
echo ""

# Step 3: Export results
echo ">>> Step 3: Exporting SLAM results..."
python "$SCRIPT_DIR/03_export_slam_results.py" \
    "$OUTPUT_DIR/slam/"
echo ""

# Step 4: Register secondary LiDARs
echo ">>> Step 4: Registering secondary LiDARs..."
for lidar_dir in "$RECORDING_DIR/raw_pointclouds"/*/; do
    lidar_name=$(basename "$lidar_dir")
    [ "$lidar_name" = "$PRIMARY_LIDAR" ] && continue

    echo "  Registering: $lidar_name"
    python "$SCRIPT_DIR/04_register_secondary.py" \
        --primary-map "$OUTPUT_DIR/slam/global_map.pcd" \
        --trajectory "$OUTPUT_DIR/slam/trajectory.txt" \
        --secondary-dir "$lidar_dir" \
        --initial-guess "$RECORDING_DIR/application.yaml" \
        --method icp \
        --output-dir "$OUTPUT_DIR/registration/$lidar_name/"
    echo ""
done

# Step 5: Solve extrinsics
echo ">>> Step 5: Solving extrinsics..."
mkdir -p "$OUTPUT_DIR/calibration"
python "$SCRIPT_DIR/05_solve_extrinsic.py" \
    --registration-dir "$OUTPUT_DIR/registration/" \
    --primary-lidar "$PRIMARY_LIDAR" \
    --output "$OUTPUT_DIR/calibration/extrinsics.yaml"
echo ""

echo "=== Pipeline Complete ==="
echo "Results: $OUTPUT_DIR/calibration/extrinsics.yaml"
```

- [ ] **Step 2: Make executable and commit**

```bash
chmod +x scripts/run_all.sh
git add scripts/run_all.sh
git commit -m "feat: add run_all.sh pipeline runner"
```

---

## Task 10: Final Integration — requirements.txt + README update

**Files:**
- Create: `requirements.txt`
- Modify: `scripts/README.md`

- [ ] **Step 1: Create requirements.txt**

```
numpy>=1.21
scipy>=1.7
open3d>=0.17
pyyaml>=6.0
```

- [ ] **Step 2: Update scripts/README.md**

```markdown
# scripts

SlamHub pipeline scripts for single-LiDAR SLAM → cross-LiDAR calibration.

## Pipeline Steps

| Step | Script | Description | Requires ROS |
|------|--------|-------------|:---:|
| 01 | `01_convert_to_rosbag.py` | Raw PCD+IMU → rosbag | Yes |
| 02 | `02_run_slam.py` | Run FAST-LIO2 | Yes |
| 03 | `03_export_slam_results.py` | Export map + trajectory + frames | No |
| 04 | `04_register_secondary.py` | Register secondary LiDARs | No |
| 05 | `05_solve_extrinsic.py` | Solve calibrated extrinsics | No |

## Quick Start

```bash
# Full pipeline (inside Docker with ROS)
./run_all.sh /path/to/recording remote_front_left_pointcloud

# Or run steps individually
python 01_convert_to_rosbag.py /path/to/recording --list-lidars
python 01_convert_to_rosbag.py /path/to/recording --lidar remote_front_left_pointcloud -o output/bags/primary.bag
```

## Visualization

```bash
python viz/show_trajectory.py output/slam/trajectory.txt
python viz/show_map.py output/slam/global_map.pcd --color-by height
python viz/show_registration.py output/slam/global_map.pcd --secondary output/registration/flash_front/ --frame 50
```

All viz scripts support `--save <path.png>` for headless rendering.

## Dependencies

```bash
pip install -r requirements.txt
```

Steps 01-02 require ROS1 Noetic (run inside FAST_LIO Docker container).
Steps 03-05 and viz/ are ROS-independent.
```

- [ ] **Step 3: Commit**

```bash
git add requirements.txt scripts/README.md
git commit -m "docs: add requirements.txt and update scripts README"
```
