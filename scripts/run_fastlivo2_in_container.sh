#!/bin/bash
# Orchestrate FAST-LIVO2 in the GHCR image.
# Mounts:
#   /data       — recording dir (read-only)
#   /workspace  — SlamHub repo (scripts, config)
#   /output     — results out
#
# This is the LIVO2 analog of run_fastlio_in_container.sh. Differences:
#   - Bag includes images (LIVO uses LiDAR + IMU + camera fusion)
#   - Two YAML configs: main (mapping_at128p.yaml) + camera (camera_<frame>.yaml)
#   - Per-recording configs auto-generated from application.yaml
set -e
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

RECORDING=/data
LIDAR=${LIDAR:-remote_front_left_pointcloud}
CAMERA_FRAME=${CAMERA_FRAME:-FRAME_CAMERA_TRAFFIC_FRONT}
BAG=/output/${LIDAR}_livo.bag

mkdir -p /output

echo "=== Step 1: Convert PCD+IMU+images → ROS bag ==="
# Validate cached bag with `rosbag info` — a partial/unindexed file from an
# interrupted previous run will silently break Step 6 (rosbag play) otherwise.
need_convert=1
if [ -f "$BAG" ]; then
    if rosbag info "$BAG" >/dev/null 2>&1; then
        echo "Bag $BAG already exists (valid) — skipping conversion."
        need_convert=0
    else
        echo "Bag $BAG exists but is corrupt/unindexed — regenerating."
        rm -f "$BAG"
    fi
fi
if [ "$need_convert" = "1" ]; then
    python3 /workspace/scripts/convert_to_rosbag_livo.py "$RECORDING" \
        --lidar "$LIDAR" --camera-frame "$CAMERA_FRAME" -o "$BAG" \
        --scan-line 128 --vfov-min -13 --vfov-max 14
fi

echo ""
echo "=== Step 2: Generate per-recording configs ==="
mkdir -p /catkin_ws/src/FAST-LIVO2/config_runtime
python3 /workspace/scripts/gen_fastlivo2_config.py "$RECORDING" \
    /catkin_ws/src/FAST-LIVO2/config_runtime \
    --camera-frame "$CAMERA_FRAME" --lidar-frame FRAME_LIDAR_REMOTE_FRONT_LEFT
ls -la /catkin_ws/src/FAST-LIVO2/config_runtime/

echo ""
echo "=== Step 3: Write headless launch ==="
cat > /catkin_ws/src/FAST-LIVO2/launch/run_at128p.launch <<EOF
<launch>
    <rosparam command="load" file="/catkin_ws/src/FAST-LIVO2/config_runtime/at128p.yaml" />

    <node pkg="fast_livo" type="fastlivo_mapping" name="laserMapping" output="screen">
        <rosparam file="/catkin_ws/src/FAST-LIVO2/config_runtime/camera_${CAMERA_FRAME,,}.yaml" />
    </node>
</launch>
EOF

echo ""
echo "=== Step 4: Start roscore ==="
roscore &
ROSCORE_PID=$!
sleep 3

echo ""
echo "=== Step 5: Launch FAST-LIVO2 ==="
roslaunch fast_livo run_at128p.launch &
SLAM_PID=$!
sleep 5

echo ""
echo "=== Step 6: Play bag ==="
BAG_DURATION=$(rosbag info "$BAG" | grep duration | head -1)
echo "Bag info: $BAG_DURATION"
rosbag play --clock "$BAG"

echo ""
echo "=== Step 7: Wait for processing to flush ==="
sleep 15

echo ""
echo "=== Step 8: Shut down ==="
rosnode kill /laserMapping 2>/dev/null || true
sleep 8
kill $SLAM_PID 2>/dev/null || true
kill $ROSCORE_PID 2>/dev/null || true
sleep 3

echo ""
echo "=== Step 9: Collect outputs ==="
# FAST-LIVO2 with our config (`pcd_save_en: true`) writes the dense map to
# /catkin_ws/src/FAST-LIVO2/Log/pcd/all_raw_points.pcd
#  + an already-downsampled all_downsampled_points.pcd (way too sparse for B2)
# We use all_raw_points.pcd as scans.pcd and let prep_fastlivo2_for_b2.py
# downsample it to 0.3 m for the cross-LiDAR ICP target.
PCD_LOG_DIR=/catkin_ws/src/FAST-LIVO2/Log/pcd
if [ -f "$PCD_LOG_DIR/all_raw_points.pcd" ]; then
    cp -v "$PCD_LOG_DIR/all_raw_points.pcd" /output/scans.pcd
fi
# Older FAST-LIO style PCD path, kept as fallback
PCD_DIR=/catkin_ws/src/FAST-LIVO2/PCD
[ -d "$PCD_DIR" ] && cp -v "$PCD_DIR"/*.pcd /output/ 2>/dev/null || true

# FAST-LIVO2 with `evo.pose_output_en: true` + `seq_name: TEEMO_AT128P` writes
# TUM-format trajectory at Log/result/TEEMO_AT128P.txt
# (time x y z qx qy qz qw, T_world_lidar). Copy as the canonical
# trajectory_lidar.txt for downstream B2 processing.
RESULT_DIR=/catkin_ws/src/FAST-LIVO2/Log/result
if [ -f "$RESULT_DIR/TEEMO_AT128P.txt" ]; then
    cp -v "$RESULT_DIR/TEEMO_AT128P.txt" /output/trajectory_lidar.txt
    echo "  trajectory_lidar.txt: $(wc -l < /output/trajectory_lidar.txt) poses"
fi

# Per-frame lidar_poses.txt (used internally) — also useful for debugging.
[ -f "$PCD_LOG_DIR/lidar_poses.txt" ] && cp -v "$PCD_LOG_DIR/lidar_poses.txt" /output/

# Save full Log/ tree for debugging (mat_pre / mat_out / image_poses etc.)
mkdir -p /output/Log
cp -r /catkin_ws/src/FAST-LIVO2/Log/. /output/Log/ 2>/dev/null || true

ls -la /output/

echo ""
echo "Done."
