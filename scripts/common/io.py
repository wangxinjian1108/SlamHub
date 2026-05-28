"""IO utilities for PCD point clouds, TUM trajectories, and YAML configs."""

import struct
from pathlib import Path
from typing import Union

import numpy as np
import yaml


def read_pcd(path: Union[str, Path]) -> np.ndarray:
    """Read a PCD file and return points as (N,3) or (N,4) float32 array.

    Supports binary and ascii DATA formats. If an intensity field is present,
    returns (N,4) with the fourth column being intensity.
    """
    path = Path(path)
    fields = []
    sizes = []
    types = []
    counts = []
    num_points = 0
    data_format = "ascii"
    header_end = 0

    with open(path, "rb") as f:
        while True:
            line = f.readline()
            header_end = f.tell()
            line_str = line.decode("ascii", errors="ignore").strip()

            if line_str.startswith("FIELDS"):
                fields = line_str.split()[1:]
            elif line_str.startswith("SIZE"):
                sizes = [int(x) for x in line_str.split()[1:]]
            elif line_str.startswith("TYPE"):
                types = line_str.split()[1:]
            elif line_str.startswith("COUNT"):
                counts = [int(x) for x in line_str.split()[1:]]
            elif line_str.startswith("POINTS"):
                num_points = int(line_str.split()[1])
            elif line_str.startswith("DATA"):
                data_format = line_str.split()[1].lower()
                break

    # Determine which columns to extract
    has_intensity = "intensity" in fields
    ncols = 4 if has_intensity else 3

    if data_format == "binary":
        # Calculate point size from sizes and counts
        if not counts:
            counts = [1] * len(fields)
        point_size = sum(s * c for s, c in zip(sizes, counts))

        with open(path, "rb") as f:
            f.seek(header_end)
            raw = f.read(num_points * point_size)

        # Build dtype from header
        dtype_map = {"F": "f", "U": "u", "I": "i"}
        dt_fields = []
        for i, (field, size, typ) in enumerate(zip(fields, sizes, types)):
            count = counts[i] if i < len(counts) else 1
            dt_char = dtype_map.get(typ, "f")
            dt = np.dtype(f"<{dt_char}{size}")
            if count == 1:
                dt_fields.append((field, dt))
            else:
                dt_fields.append((field, dt, (count,)))

        structured = np.frombuffer(raw, dtype=np.dtype(dt_fields), count=num_points)

        points = np.zeros((num_points, ncols), dtype=np.float32)
        points[:, 0] = structured["x"].astype(np.float32)
        points[:, 1] = structured["y"].astype(np.float32)
        points[:, 2] = structured["z"].astype(np.float32)
        if has_intensity:
            points[:, 3] = structured["intensity"].astype(np.float32)

    else:
        # ASCII format
        rows = []
        with open(path, "r") as f:
            # Skip header
            for line in f:
                if line.strip().startswith("DATA"):
                    break
            for line in f:
                parts = line.strip().split()
                if len(parts) >= len(fields):
                    rows.append([float(x) for x in parts])

        all_data = np.array(rows, dtype=np.float32)
        points = np.zeros((len(rows), ncols), dtype=np.float32)

        x_idx = fields.index("x")
        y_idx = fields.index("y")
        z_idx = fields.index("z")
        points[:, 0] = all_data[:, x_idx]
        points[:, 1] = all_data[:, y_idx]
        points[:, 2] = all_data[:, z_idx]
        if has_intensity:
            i_idx = fields.index("intensity")
            points[:, 3] = all_data[:, i_idx]

    return points


def write_pcd(path: Union[str, Path], points: np.ndarray) -> None:
    """Write points to a binary PCD v0.7 file.

    Args:
        path: Output file path.
        points: (N,3) or (N,4) float32 array. If 4 columns, the fourth is intensity.
    """
    path = Path(path)
    points = np.asarray(points, dtype=np.float32)
    n = points.shape[0]
    has_intensity = points.shape[1] == 4

    if has_intensity:
        fields = "x y z intensity"
        sizes = "4 4 4 4"
        types = "F F F F"
        counts = "1 1 1 1"
        width = 4
    else:
        fields = "x y z"
        sizes = "4 4 4"
        types = "F F F"
        counts = "1 1 1"
        width = 3

    point_size = width * 4  # each field is 4 bytes (float32)

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
        f.write(points.tobytes())


def read_trajectory_tum(path: Union[str, Path]) -> np.ndarray:
    """Read a TUM-format trajectory file.

    Returns (N,8) float64 array: [timestamp tx ty tz qx qy qz qw].
    Lines starting with '#' are skipped.
    """
    path = Path(path)
    rows = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) >= 8:
                rows.append([float(x) for x in parts[:8]])

    return np.array(rows, dtype=np.float64)


def write_trajectory_tum(path: Union[str, Path], poses: np.ndarray) -> None:
    """Write poses to a TUM-format trajectory file.

    Args:
        path: Output file path.
        poses: (N,8) array with columns [timestamp tx ty tz qx qy qz qw].
    """
    path = Path(path)
    poses = np.asarray(poses, dtype=np.float64)

    with open(path, "w") as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for row in poses:
            f.write(
                f"{row[0]:.6f} {row[1]:.6f} {row[2]:.6f} {row[3]:.6f} "
                f"{row[4]:.6f} {row[5]:.6f} {row[6]:.6f} {row[7]:.6f}\n"
            )


def read_yaml(path: Union[str, Path]) -> dict:
    """Read a YAML file and return its contents as a dictionary."""
    path = Path(path)
    with open(path, "r") as f:
        return yaml.safe_load(f)


def write_yaml(path: Union[str, Path], data: dict) -> None:
    """Write a dictionary to a YAML file."""
    path = Path(path)
    with open(path, "w") as f:
        yaml.dump(data, f, default_flow_style=False)
