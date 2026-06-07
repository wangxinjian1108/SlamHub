#!/usr/bin/env python3
"""Convert TEEMO PCD + IMU + camera frames → ROS1 rosbag for FAST-LIVO2.

FAST-LIVO2 expects:
  /velodyne_points (sensor_msgs/PointCloud2)  — same Velodyne x,y,z,intensity,time,ring format as our LIO bag
  /imu/data        (sensor_msgs/Imu)           — same as LIO bag
  /camera/image    (sensor_msgs/Image OR sensor_msgs/CompressedImage)
                                                — front-facing image stream, BGR8

Why we re-implement instead of just adding `add image stream` to
convert_to_rosbag_velodyne.py:
  - LIVO needs accel sign FOR THE GTSAM-INHERITING PREINTEGRATOR (none here —
    FAST-LIVO2 uses an ESKF, accel sign convention matches FAST-LIO so no negate)
  - Image timestamp alignment to LiDAR / IMU is the new variable
  - Image res reduction (FAST-LIVO2 internal scale=0.5 already handles 1920×1080)

Usage (inside the GHCR FAST-LIVO2 container):
    python3 convert_to_rosbag_livo.py <recording_dir> \
        --lidar remote_front_left_pointcloud \
        --camera-frame FRAME_CAMERA_TRAFFIC_FRONT \
        -o out.bag
"""
import argparse
import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from convert_to_rosbag_velodyne import parse_pcd_binary, derive_ring

CAM_DIR_MAP = {
    "FRAME_CAMERA_TRAFFIC_FRONT":  "TRAFFIC_FRONT",
    "FRAME_CAMERA_TRAFFIC_LEFT":   "TRAFFIC_LEFT",
    "FRAME_CAMERA_TRAFFIC_RIGHT":  "TRAFFIC_RIGHT",
    "FRAME_CAMERA360_FRONT_LEFT":  "CAM360_F_LEFT",
    "FRAME_CAMERA360_FRONT_RIGHT": "CAM360_F_RIGHT",
    "FRAME_CAMERA360_FLEFT":       "CAM360_FLEFT",
    "FRAME_CAMERA360_FRIGHT":      "CAM360_FRIGHT",
    "FRAME_CAMERA360_BLEFT":       "CAM360_BLEFT",
    "FRAME_CAMERA360_BRIGHT":      "CAM360_BRIGHT",
    "FRAME_CAMERA360_REAR":        "CAM360_REAR",
    "FRAME_CAMERA_MONITOR_FRONT":  "MONITOR_FRONT",
    "FRAME_CAMERA_MONITOR_REAR":   "MONITOR_REAR",
}


def convert(recording_dir, lidar_name, camera_frame, output_path,
            scan_line, vfov_min, vfov_max,
            lidar_topic, imu_topic, image_topic):
    import rosbag
    import rospy
    from sensor_msgs.msg import PointCloud2, PointField, Imu, Image
    from std_msgs.msg import Header

    # --- Resolve image directory ---
    img_subdir = CAM_DIR_MAP.get(camera_frame)
    if img_subdir is None:
        sys.exit(f"camera frame {camera_frame} not in CAM_DIR_MAP")
    img_dir = recording_dir / img_subdir
    if not img_dir.exists():
        sys.exit(f"image dir {img_dir} not found")

    lidar_dir = recording_dir / "raw_pointclouds" / lidar_name
    imu_path = recording_dir / "imu.csv"
    pcd_files = sorted(lidar_dir.glob("*.pcd"))
    img_files = sorted(img_dir.glob("*.png"))
    print(f"  PCDs: {len(pcd_files)}, IMU csv exists, images: {len(img_files)}")

    # Velodyne point format
    point_fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="time", offset=16, datatype=PointField.FLOAT32, count=1),
        PointField(name="ring", offset=20, datatype=PointField.UINT16, count=1),
    ]
    point_step = 24
    pt_dtype = np.dtype({
        "names": ["x", "y", "z", "intensity", "time", "ring"],
        "formats": ["f4", "f4", "f4", "f4", "f4", "u2"],
        "offsets": [0, 4, 8, 12, 16, 20],
        "itemsize": point_step,
    })

    import cv2

    with rosbag.Bag(str(output_path), "w") as bag:
        # --- IMU ---
        imu_count = 0
        with open(imu_path) as f:
            for row in csv.DictReader(f):
                ts_ns = int(float(row["time"]) * 1e6)
                msg = Imu()
                msg.header = Header()
                msg.header.stamp = rospy.Time(ts_ns // 1_000_000_000, ts_ns % 1_000_000_000)
                msg.header.frame_id = "imu_link"
                deg2rad = np.pi / 180.0
                msg.linear_acceleration.x = float(row["gx"])
                msg.linear_acceleration.y = float(row["gy"])
                msg.linear_acceleration.z = float(row["gz"])
                msg.angular_velocity.x = float(row["wx"]) * deg2rad
                msg.angular_velocity.y = float(row["wy"]) * deg2rad
                msg.angular_velocity.z = float(row["wz"]) * deg2rad
                msg.orientation.w = 1.0
                msg.orientation_covariance[0] = -1.0
                bag.write(imu_topic, msg, msg.header.stamp)
                imu_count += 1
        print(f"  wrote {imu_count} IMU messages")

        # --- LiDAR ---
        for i, pcd_file in enumerate(pcd_files):
            frame_ts_ns = int(pcd_file.stem)
            data = parse_pcd_binary(pcd_file)
            mask = ~np.isnan(data["x"])
            d = data[mask]
            n = len(d)
            if n == 0:
                continue
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
            t0_ns = int(round(t0 * 1e9))
            msg = PointCloud2()
            msg.header = Header()
            msg.header.stamp = rospy.Time(t0_ns // 1_000_000_000, t0_ns % 1_000_000_000)
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
                print(f"    LiDAR {i+1}/{len(pcd_files)}")

        # --- Camera ---
        cam_count = 0
        for img_file in img_files:
            # Name format: 0000_xxx_<ts_ns>.png
            ts_ns = int(img_file.stem.split("_")[-1])
            img = cv2.imread(str(img_file), cv2.IMREAD_COLOR)
            if img is None:
                continue
            msg = Image()
            msg.header = Header()
            msg.header.stamp = rospy.Time(ts_ns // 1_000_000_000, ts_ns % 1_000_000_000)
            msg.header.frame_id = "camera"
            msg.height, msg.width = img.shape[:2]
            msg.encoding = "bgr8"
            msg.is_bigendian = 0
            msg.step = 3 * msg.width
            msg.data = img.tobytes()
            bag.write(image_topic, msg, msg.header.stamp)
            cam_count += 1
            if cam_count % 100 == 0:
                print(f"    Image {cam_count}/{len(img_files)}")
        print(f"  wrote {cam_count} image messages")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("recording_dir", type=Path)
    p.add_argument("--lidar", default="remote_front_left_pointcloud")
    p.add_argument("--camera-frame", default="FRAME_CAMERA_TRAFFIC_FRONT",
                   help="Which TEEMO camera frame to use (default front-facing pinhole)")
    p.add_argument("-o", "--output", type=Path, required=True)
    p.add_argument("--scan-line", type=int, default=128)
    p.add_argument("--vfov-min", type=float, default=-13.0)
    p.add_argument("--vfov-max", type=float, default=14.0)
    p.add_argument("--lidar-topic", default="/velodyne_points")
    p.add_argument("--imu-topic", default="/imu/data")
    p.add_argument("--image-topic", default="/camera/image")
    args = p.parse_args()
    convert(args.recording_dir, args.lidar, args.camera_frame, args.output,
            args.scan_line, args.vfov_min, args.vfov_max,
            args.lidar_topic, args.imu_topic, args.image_topic)
    print(f"Done. Bag at {args.output}")


if __name__ == "__main__":
    main()
