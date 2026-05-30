#!/usr/bin/env python3
"""
Convert raw PCD + IMU to ROS1 bag in Velodyne format for FAST-LIO2.

FAST-LIO2's velodyne_handler expects PointCloud2 with fields:
    x, y, z (float32), intensity (float32), time (float32, per-point offset in seconds), ring (uint16)

This script derives:
- ring: from vertical angle, mapped to [0, scan_line)
- time: per-point offset from frame start (seconds), computed from the absolute
  per-point timestamp in the source PCD 'time' field

IMU csv columns: time(ms), gx,gy,gz (accel m/s^2), wx,wy,wz (gyro rad/s)

Usage (inside FAST_LIO docker container):
    python convert_to_rosbag_velodyne.py <recording_dir> --lidar remote_front_left_pointcloud -o out.bag
"""

import argparse
import csv
import sys
from pathlib import Path

import numpy as np


def parse_pcd_binary(pcd_path):
    with open(pcd_path, "rb") as f:
        header_lines = []
        while True:
            line = f.readline().decode("ascii", errors="ignore").strip()
            header_lines.append(line)
            if line.startswith("DATA"):
                break
        fields, sizes, types, num_points = [], [], [], 0
        for line in header_lines:
            if line.startswith("FIELDS"):
                fields = line.split()[1:]
            elif line.startswith("SIZE"):
                sizes = [int(x) for x in line.split()[1:]]
            elif line.startswith("TYPE"):
                types = line.split()[1:]
            elif line.startswith("POINTS"):
                num_points = int(line.split()[1])
        dtype_map = {"F": "f", "I": "i", "U": "u"}
        dt = np.dtype([(n, f"{dtype_map[t]}{s}") for n, t, s in zip(fields, types, sizes)])
        data = np.frombuffer(f.read(num_points * dt.itemsize), dtype=dt, count=num_points)
    return data


def derive_ring(x, y, z, scan_line, vfov_min, vfov_max):
    """Map vertical angle to ring index [0, scan_line)."""
    r = np.sqrt(x * x + y * y)
    vert = np.degrees(np.arctan2(z, r))
    vert = np.clip(vert, vfov_min, vfov_max)
    ring = ((vert - vfov_min) / (vfov_max - vfov_min) * (scan_line - 1)).astype(np.uint16)
    return ring


def convert(recording_dir, lidar_name, output_path, scan_line, vfov_min, vfov_max,
            lidar_topic, imu_topic):
    import rosbag
    import rospy
    from sensor_msgs.msg import PointCloud2, PointField, Imu
    from std_msgs.msg import Header

    lidar_dir = recording_dir / "raw_pointclouds" / lidar_name
    imu_path = recording_dir / "imu.csv"
    pcd_files = sorted(lidar_dir.glob("*.pcd"))
    print(f"Found {len(pcd_files)} PCD frames, scan_line={scan_line}, vfov=[{vfov_min},{vfov_max}]")

    # Velodyne point format: x,y,z,intensity,time,ring
    point_fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="time", offset=16, datatype=PointField.FLOAT32, count=1),
        PointField(name="ring", offset=20, datatype=PointField.UINT16, count=1),
    ]
    point_step = 24  # 5*float32 + uint16 padded to 24
    pt_dtype = np.dtype({
        "names": ["x", "y", "z", "intensity", "time", "ring"],
        "formats": ["f4", "f4", "f4", "f4", "f4", "u2"],
        "offsets": [0, 4, 8, 12, 16, 20],
        "itemsize": point_step,
    })

    with rosbag.Bag(str(output_path), "w") as bag:
        # --- IMU messages ---
        imu_count = 0
        with open(imu_path) as f:
            for row in csv.DictReader(f):
                ts_ns = int(float(row["time"]) * 1e6)  # ms -> ns
                msg = Imu()
                msg.header = Header()
                msg.header.stamp = rospy.Time(ts_ns // 1_000_000_000, ts_ns % 1_000_000_000)
                msg.header.frame_id = "imu_link"
                # Source gyro (wx,wy,wz) is in deg/s despite the SI-looking column
                # names. RMS over the bag is ~214 deg/s — interpreting these as rad/s
                # would mean the car yaws several times per second. Convert to rad/s.
                # Accel (gx,gy,gz) is m/s^2, "specific force" convention (gz≈-9.8 at
                # rest, Z-up): FAST-LIO handles this — leave as-is.
                deg2rad = np.pi / 180.0
                msg.linear_acceleration.x = float(row["gx"])
                msg.linear_acceleration.y = float(row["gy"])
                msg.linear_acceleration.z = float(row["gz"])
                msg.angular_velocity.x = float(row["wx"]) * deg2rad
                msg.angular_velocity.y = float(row["wy"]) * deg2rad
                msg.angular_velocity.z = float(row["wz"]) * deg2rad
                msg.orientation_covariance[0] = -1.0
                bag.write(imu_topic, msg, msg.header.stamp)
                imu_count += 1
        print(f"Wrote {imu_count} IMU messages")

        # --- LiDAR messages ---
        for i, pcd_file in enumerate(pcd_files):
            frame_ts_ns = int(pcd_file.stem)
            data = parse_pcd_binary(pcd_file)
            mask = ~np.isnan(data["x"])
            d = data[mask]
            n = len(d)
            if n == 0:
                continue

            # per-point relative time (seconds) from frame start
            abs_t = d["time"].astype(np.float64)
            t0 = abs_t.min()
            rel_time = (abs_t - t0).astype(np.float32)

            ring = derive_ring(d["x"], d["y"], d["z"], scan_line, vfov_min, vfov_max)

            buf = np.zeros(n, dtype=pt_dtype)
            buf["x"] = d["x"]
            buf["y"] = d["y"]
            buf["z"] = d["z"]
            buf["intensity"] = d["intensity"]
            buf["time"] = rel_time
            buf["ring"] = ring

            msg = PointCloud2()
            msg.header = Header()
            msg.header.stamp = rospy.Time(frame_ts_ns // 1_000_000_000, frame_ts_ns % 1_000_000_000)
            msg.header.frame_id = "lidar_link"
            msg.fields = point_fields
            msg.height = 1
            msg.width = n
            msg.point_step = point_step
            msg.row_step = point_step * n
            msg.is_bigendian = False
            msg.is_dense = True
            msg.data = buf.tobytes()
            bag.write(lidar_topic, msg, msg.header.stamp)

            if (i + 1) % 100 == 0:
                print(f"  {i+1}/{len(pcd_files)} frames")

    print(f"Done. Bag written to {output_path}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("recording_dir", type=Path)
    p.add_argument("--lidar", required=True)
    p.add_argument("-o", "--output", type=Path, required=True)
    p.add_argument("--scan-line", type=int, default=128)
    p.add_argument("--vfov-min", type=float, default=-13.0)
    p.add_argument("--vfov-max", type=float, default=14.0)
    p.add_argument("--lidar-topic", default="/velodyne_points")
    p.add_argument("--imu-topic", default="/imu/data")
    args = p.parse_args()
    convert(args.recording_dir, args.lidar, args.output, args.scan_line,
            args.vfov_min, args.vfov_max, args.lidar_topic, args.imu_topic)


if __name__ == "__main__":
    main()
