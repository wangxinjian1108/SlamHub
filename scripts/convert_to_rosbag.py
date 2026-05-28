#!/usr/bin/env python3
"""
Convert raw PCD + IMU data to ROS1 bag for FAST-LIO2 and other LIO methods.

Supports selecting any LiDAR from the raw_pointclouds directory.

Usage:
    python convert_to_rosbag.py /path/to/recording --lidar remote_front_left_pointcloud
    python convert_to_rosbag.py /path/to/recording --lidar flash_front_pointcloud --output output.bag
    python convert_to_rosbag.py /path/to/recording --list-lidars
"""

import argparse
import csv
import sys
from pathlib import Path

import numpy as np
import yaml


def list_available_lidars(recording_dir: Path) -> list:
    raw_dir = recording_dir / "raw_pointclouds"
    if not raw_dir.exists():
        return []
    return sorted([d.name for d in raw_dir.iterdir() if d.is_dir()])


def parse_pcd_binary(pcd_path: Path) -> np.ndarray:
    with open(pcd_path, "rb") as f:
        header_lines = []
        while True:
            line = f.readline().decode("ascii", errors="ignore").strip()
            header_lines.append(line)
            if line == "DATA binary":
                break

        fields = []
        sizes = []
        types = []
        num_points = 0

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
        dt = np.dtype(
            [(name, f"{dtype_map[t]}{s}") for name, t, s in zip(fields, types, sizes)]
        )

        data = np.frombuffer(f.read(num_points * dt.itemsize), dtype=dt, count=num_points)

    return data


def get_extrinsic(application_yaml: Path, lidar_name: str) -> dict:
    frame_map = {
        "remote_front_left_pointcloud": "FRAME_LIDAR_REMOTE_FRONT_LEFT",
        "remote_front_right_pointcloud": "FRAME_LIDAR_REMOTE_FRONT_RIGHT",
        "flash_front_pointcloud": "FRAME_LIDAR_FLASH_FRONT",
        "flash_rear_pointcloud": "FRAME_LIDAR_FLASH_REAR",
    }

    frame_name = frame_map.get(lidar_name)
    if not frame_name:
        return None

    with open(application_yaml) as f:
        config = yaml.safe_load(f)

    calibrations = config["vehicle"]["calibration"]["sensor_calibration"]
    for cal in calibrations:
        if cal["source"] == frame_name:
            t = cal["transformation"]
            return {
                "frame": frame_name,
                "x": t[0], "y": t[1], "z": t[2],
                "roll": t[3], "pitch": t[4], "yaw": t[5],
            }
    return None


def convert(
    recording_dir: Path,
    lidar_name: str,
    output_path: Path,
    lidar_topic: str,
    imu_topic: str,
    lidar_frame: str,
    imu_frame: str,
):
    try:
        import rosbag
        import rospy
        from sensor_msgs.msg import PointCloud2, PointField, Imu
        from std_msgs.msg import Header
    except ImportError:
        print("Error: ROS1 Python packages not found.")
        print("Run this script inside the FAST_LIO docker container or source ROS Noetic.")
        sys.exit(1)

    lidar_dir = recording_dir / "raw_pointclouds" / lidar_name
    imu_path = recording_dir / "imu.csv"

    if not lidar_dir.exists():
        print(f"Error: LiDAR directory not found: {lidar_dir}")
        sys.exit(1)
    if not imu_path.exists():
        print(f"Error: IMU file not found: {imu_path}")
        sys.exit(1)

    pcd_files = sorted(lidar_dir.glob("*.pcd"))
    print(f"Found {len(pcd_files)} PCD frames from '{lidar_name}'")

    imu_rows = []
    with open(imu_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            imu_rows.append(row)
    print(f"Found {len(imu_rows)} IMU measurements")

    extrinsic = get_extrinsic(recording_dir / "application.yaml", lidar_name)
    if extrinsic:
        print(f"Extrinsic ({extrinsic['frame']} -> base_link): "
              f"t=[{extrinsic['x']:.3f}, {extrinsic['y']:.3f}, {extrinsic['z']:.3f}] "
              f"rpy=[{extrinsic['roll']:.4f}, {extrinsic['pitch']:.4f}, {extrinsic['yaw']:.4f}]")

    print(f"Writing rosbag to: {output_path}")
    with rosbag.Bag(str(output_path), "w") as bag:
        for i, pcd_file in enumerate(pcd_files):
            timestamp_ns = int(pcd_file.stem)
            pcd_data = parse_pcd_binary(pcd_file)

            # Build PointCloud2 message
            msg = PointCloud2()
            sec = timestamp_ns // 1_000_000_000
            nsec = timestamp_ns % 1_000_000_000
            msg.header = Header()
            msg.header.stamp = rospy.Time(sec, nsec)
            msg.header.frame_id = lidar_frame

            valid_mask = ~np.isnan(pcd_data["x"])
            valid_data = pcd_data[valid_mask]
            num_points = len(valid_data)

            buffer = np.zeros(num_points, dtype=np.dtype([
                ("x", "f4"), ("y", "f4"), ("z", "f4"), ("intensity", "f4")
            ]))
            buffer["x"] = valid_data["x"]
            buffer["y"] = valid_data["y"]
            buffer["z"] = valid_data["z"]
            buffer["intensity"] = valid_data["intensity"]

            msg.fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            msg.height = 1
            msg.width = num_points
            msg.point_step = 16
            msg.row_step = 16 * num_points
            msg.is_bigendian = False
            msg.is_dense = True
            msg.data = buffer.tobytes()

            bag.write(lidar_topic, msg, msg.header.stamp)

            if (i + 1) % 100 == 0:
                print(f"  Processed {i + 1}/{len(pcd_files)} frames")

        for row in imu_rows:
            imu_msg = Imu()
            timestamp_ms = float(row["time"])
            timestamp_ns = int(timestamp_ms * 1_000_000)
            sec = timestamp_ns // 1_000_000_000
            nsec = timestamp_ns % 1_000_000_000

            imu_msg.header = Header()
            imu_msg.header.stamp = rospy.Time(sec, nsec)
            imu_msg.header.frame_id = imu_frame

            imu_msg.linear_acceleration.x = float(row["gx"])
            imu_msg.linear_acceleration.y = float(row["gy"])
            imu_msg.linear_acceleration.z = float(row["gz"])
            imu_msg.angular_velocity.x = float(row["wx"])
            imu_msg.angular_velocity.y = float(row["wy"])
            imu_msg.angular_velocity.z = float(row["wz"])
            imu_msg.orientation_covariance[0] = -1.0

            bag.write(imu_topic, imu_msg, imu_msg.header.stamp)

    print(f"Done. Bag written with {len(pcd_files)} point clouds + {len(imu_rows)} IMU messages.")


def main():
    parser = argparse.ArgumentParser(
        description="Convert raw PCD + IMU to ROS1 bag for LIO methods"
    )
    parser.add_argument("recording_dir", type=Path, help="Path to recording directory")
    parser.add_argument("--lidar", type=str, help="LiDAR subdirectory name under raw_pointclouds/")
    parser.add_argument("--output", "-o", type=Path, default=None, help="Output bag path")
    parser.add_argument("--lidar-topic", default="/points_raw", help="PointCloud2 topic name")
    parser.add_argument("--imu-topic", default="/imu/data", help="IMU topic name")
    parser.add_argument("--lidar-frame", default="lidar", help="LiDAR frame_id")
    parser.add_argument("--imu-frame", default="imu", help="IMU frame_id")
    parser.add_argument("--list-lidars", action="store_true", help="List available LiDARs and exit")

    args = parser.parse_args()

    if args.list_lidars:
        lidars = list_available_lidars(args.recording_dir)
        if not lidars:
            print("No LiDARs found in raw_pointclouds/")
        else:
            print("Available LiDARs:")
            for name in lidars:
                count = len(list((args.recording_dir / "raw_pointclouds" / name).glob("*.pcd")))
                print(f"  {name} ({count} frames)")
        return

    if not args.lidar:
        parser.error("--lidar is required (use --list-lidars to see options)")

    if args.output is None:
        recording_name = args.recording_dir.name
        args.output = Path(f"{recording_name}_{args.lidar}.bag")

    convert(
        recording_dir=args.recording_dir,
        lidar_name=args.lidar,
        output_path=args.output,
        lidar_topic=args.lidar_topic,
        imu_topic=args.imu_topic,
        lidar_frame=args.lidar_frame,
        imu_frame=args.imu_frame,
    )


if __name__ == "__main__":
    main()
