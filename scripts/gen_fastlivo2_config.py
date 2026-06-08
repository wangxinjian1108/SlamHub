#!/usr/bin/env python3
"""Generate FAST-LIVO2 camera_*.yaml + main config from TEEMO application.yaml.

FAST-LIVO2's main config (e.g. avia.yaml) needs:
  - common.{img_topic, lid_topic, imu_topic}
  - extrin_calib.{Rcl, Pcl}  (LiDAR pose in camera frame, R_camera_lidar)
  - extrin_calib.{extrinsic_T, extrinsic_R}  (IMU → body offset; we set identity since our IMU IS the body)
  - preprocess.{lidar_type, scan_line, blind, ...}

FAST-LIVO2's camera config (e.g. camera_pinhole.yaml) needs:
  - cam_model: Pinhole or EquidistantCamera
  - cam_width, cam_height, scale
  - For Pinhole: cam_fx, cam_fy, cam_cx, cam_cy, cam_d0..d3 (Brown-Conrady; d0=k1, d1=k2, d2=p1, d3=p2; ignore k3 if 5-coeff)
  - For Fisheye/EquidistantCamera: cam_fx, cam_fy, cam_cx, cam_cy, k1, k2, k3, k4

We pull both intrinsics and extrinsics from the TEEMO application.yaml:
  vehicle.calibration.camera_calibration[i]: {frame, intrinsic[9], distortion[4or5], camera_model}
  vehicle.calibration.sensor_calibration[j]: {source, transformation: [x,y,z,roll,pitch,yaw]}

Coordinate convention:
  - TEEMO transformation = T_baselink_<sensor> ([x,y,z,roll,pitch,yaw])
  - FAST-LIVO2 Rcl + Pcl = T_camera_lidar (rotation of LiDAR frame as seen from camera)
"""
import argparse
import sys
from pathlib import Path
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).parent))
from common.transform import euler_to_matrix, make_homogeneous, invert_transform

# Camera frame → image dir name in TEEMO fixture layout
CAM_DIR_MAP = {
    "FRAME_CAMERA_TRAFFIC_FRONT": "TRAFFIC_FRONT",
    "FRAME_CAMERA_TRAFFIC_LEFT":  "TRAFFIC_LEFT",
    "FRAME_CAMERA_TRAFFIC_RIGHT": "TRAFFIC_RIGHT",
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

# LiDAR frame → raw_pointclouds subdir
LIDAR_FRAME_DIR = {
    "FRAME_LIDAR_REMOTE_FRONT_LEFT":  "remote_front_left_pointcloud",
    "FRAME_LIDAR_REMOTE_FRONT_RIGHT": "remote_front_right_pointcloud",
    "FRAME_LIDAR_FLASH_FRONT":        "flash_front_pointcloud",
    "FRAME_LIDAR_FLASH_REAR":         "flash_rear_pointcloud",
}


def load_calibration(application_yaml):
    cfg = yaml.safe_load(open(application_yaml))
    cams = {c["frame"]: c for c in cfg["vehicle"]["calibration"]["camera_calibration"]}
    sensors = {s["source"]: s for s in cfg["vehicle"]["calibration"]["sensor_calibration"]}
    return cams, sensors


def transformation_to_T(tf6):
    """[x, y, z, roll, pitch, yaw] -> 4x4 T_parent_child."""
    R = euler_to_matrix(tf6[3], tf6[4], tf6[5])
    return make_homogeneous(R, np.array(tf6[:3]))


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("recording", type=Path,
                   help="Recording dir containing application.yaml")
    p.add_argument("out_dir", type=Path,
                   help="Output dir for the generated yaml configs")
    p.add_argument("--camera-frame", default="FRAME_CAMERA_TRAFFIC_FRONT",
                   help="Which TEEMO camera frame to feed FAST-LIVO2 "
                        "(default: TRAFFIC_FRONT pinhole — best for forward LiDAR alignment)")
    p.add_argument("--lidar-frame", default="FRAME_LIDAR_REMOTE_FRONT_LEFT",
                   help="Primary LiDAR frame (default matches our other backends)")
    p.add_argument("--scale", type=float, default=0.5,
                   help="cam scale (FAST-LIVO2 downsamples internally for VIO; 0.5 = half res)")
    args = p.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    cams, sensors = load_calibration(args.recording / "application.yaml")

    if args.camera_frame not in cams:
        sys.exit(f"camera frame {args.camera_frame} not found in application.yaml")
    if args.camera_frame not in CAM_DIR_MAP:
        sys.exit(f"camera frame {args.camera_frame} has no image dir mapping")

    # ----- Camera intrinsics -----
    cam = cams[args.camera_frame]
    intrinsic = cam["intrinsic"]   # 9 entries (3x3)
    fx, _, cx, _, fy, cy, _, _, _ = intrinsic
    distortion = cam["distortion"]
    model = cam["camera_model"]

    # Find actual image dimensions from disk
    img_dir = args.recording / CAM_DIR_MAP[args.camera_frame]
    sample = next(img_dir.glob("*.png"), None) if img_dir.exists() else None
    cam_width, cam_height = 1920, 1080  # fallback
    if sample is not None:
        try:
            from PIL import Image
            im = Image.open(sample)
            cam_width, cam_height = im.width, im.height
        except ImportError:
            try:
                import cv2
                im = cv2.imread(str(sample), cv2.IMREAD_COLOR)
                if im is not None:
                    cam_height, cam_width = im.shape[:2]
            except Exception:
                # Final fallback: parse PNG IHDR (16 bytes after PNG signature)
                # This works without any image lib.
                with open(sample, "rb") as f:
                    data = f.read(24)
                if data[:8] == b"\x89PNG\r\n\x1a\n":
                    import struct
                    cam_width, cam_height = struct.unpack(">II", data[16:24])

    # ----- Camera extrinsic (T_lidar_camera) -----
    # TEEMO gives T_baselink_camera and T_baselink_lidar.
    # FAST-LIVO2's Rcl + Pcl = T_camera_lidar
    T_bl_cam = transformation_to_T(sensors[args.camera_frame]["transformation"])
    T_bl_lidar = transformation_to_T(sensors[args.lidar_frame]["transformation"])
    T_cam_lidar = invert_transform(T_bl_cam) @ T_bl_lidar
    Rcl = T_cam_lidar[:3, :3]
    Pcl = T_cam_lidar[:3, 3]

    # IMU offset in body frame = identity (we treat IMU == body, FAST-LIVO2 default)
    extrinsic_T = [0.0, 0.0, 0.0]
    extrinsic_R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

    # ----- Write camera_*.yaml -----
    cam_yaml = args.out_dir / f"camera_{args.camera_frame.lower()}.yaml"
    if model == "pinhole":
        # TEEMO distortion is [k1, k2, p1, p2, k3]; FAST-LIVO2 takes 4 coeffs
        # cam_d0..d3 = [k1, k2, p1, p2]. (k3 dropped — pinhole_camera in vikit
        # uses the 4-coeff Brown-Conrady model)
        with open(cam_yaml, "w") as f:
            f.write(f"# Auto-generated for {args.camera_frame}\n")
            f.write(f"cam_model: Pinhole\n")
            f.write(f"cam_width: {cam_width}\n")
            f.write(f"cam_height: {cam_height}\n")
            f.write(f"scale: {args.scale}\n")
            f.write(f"cam_fx: {fx:.6f}\n")
            f.write(f"cam_fy: {fy:.6f}\n")
            f.write(f"cam_cx: {cx:.6f}\n")
            f.write(f"cam_cy: {cy:.6f}\n")
            f.write(f"cam_d0: {distortion[0]:.6f}\n")
            f.write(f"cam_d1: {distortion[1]:.6f}\n")
            f.write(f"cam_d2: {distortion[2]:.6f}\n")
            f.write(f"cam_d3: {distortion[3]:.6f}\n")
    elif model == "fisheye":
        with open(cam_yaml, "w") as f:
            f.write(f"# Auto-generated for {args.camera_frame}\n")
            f.write(f"cam_model: EquidistantCamera\n")
            f.write(f"cam_width: {cam_width}\n")
            f.write(f"cam_height: {cam_height}\n")
            f.write(f"scale: {args.scale}\n")
            f.write(f"cam_fx: {fx:.6f}\n")
            f.write(f"cam_fy: {fy:.6f}\n")
            f.write(f"cam_cx: {cx:.6f}\n")
            f.write(f"cam_cy: {cy:.6f}\n")
            for i, k in enumerate(distortion[:4]):
                f.write(f"k{i+1}: {k:.6f}\n")
    else:
        sys.exit(f"unknown camera model {model}")
    print(f"  wrote {cam_yaml}")

    # ----- Write main config (mapping_at128p.yaml mirror of avia.yaml) -----
    mat = lambda M: ", ".join(f"{x:.6f}" for x in M.ravel())
    main_yaml = args.out_dir / "at128p.yaml"
    with open(main_yaml, "w") as f:
        f.write(f"""# Auto-generated FAST-LIVO2 main config for AT128P + {args.camera_frame}
common:
  img_topic: "/camera/image"
  lid_topic: "/velodyne_points"
  imu_topic: "/imu/data"
  img_en: 1
  lidar_en: 1
  ros_driver_bug_fix: false

extrin_calib:
  extrinsic_T: [{extrinsic_T[0]}, {extrinsic_T[1]}, {extrinsic_T[2]}]
  extrinsic_R: [{', '.join(f'{x:.6f}' for x in extrinsic_R)}]
  Rcl: [{mat(Rcl)}]
  Pcl: [{Pcl[0]:.6f}, {Pcl[1]:.6f}, {Pcl[2]:.6f}]

time_offset:
  imu_time_offset: 0.0
  img_time_offset: 0.0
  exposure_time_init: 0.0

preprocess:
  point_filter_num: 1
  filter_size_surf: 0.5
  lidar_type: 2     # 2 = Velodyne-style (matches what convert_to_rosbag_velodyne.py emits)
  scan_line: 128
  blind: 1.0

vio:
  max_iterations: 5
  outlier_threshold: 1000
  img_point_cov: 100
  patch_size: 8
  patch_pyrimid_level: 4
  normal_en: true
  raycast_en: false
  inverse_composition_en: false
  exposure_estimate_en: true
  inv_expo_cov: 0.1

imu:
  imu_en: true
  imu_int_frame: 30
  acc_cov: 0.5
  gyr_cov: 0.3
  b_acc_cov: 0.0001
  b_gyr_cov: 0.0001

lio:
  max_iterations: 5
  dept_err: 0.02
  beam_err: 0.05
  min_eigen_value: 0.0025
  voxel_size: 0.5
  max_layer: 2
  max_points_num: 50
  layer_init_num: [5, 5, 5, 5, 5]

local_map:
  map_sliding_en: true
  half_map_size: 100
  sliding_thresh: 8

uav:
  imu_rate_odom: false
  gravity_align_en: false

publish:
  dense_map_en: true
  pub_effect_point_en: false
  pub_plane_en: false
  pub_scan_num: 1
  blind_rgb_points: 0.0

evo:
  seq_name: "TEEMO_AT128P"
  pose_output_en: true

pcd_save:
  pcd_save_en: true
  interval: -1
""")
    print(f"  wrote {main_yaml}")
    print(f"  T_camera_lidar t={Pcl} (m)")


if __name__ == "__main__":
    main()
