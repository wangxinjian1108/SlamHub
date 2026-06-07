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
if [ -f "$BAG" ]; then
    echo "Bag $BAG already exists — skipping conversion."
else
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
PCD_DIR=/catkin_ws/src/FAST-LIVO2/PCD
ls -la "$PCD_DIR"/ 2>/dev/null || echo "No PCD dir"
cp -v "$PCD_DIR"/*.pcd /output/ 2>/dev/null || echo "No PCD files saved"
# Trajectory dumps
for src in /catkin_ws/src/FAST-LIVO2/Log/pos_log.txt \
           /catkin_ws/src/FAST-LIVO2/Log/traj.txt \
           ~/.ros/pos_log.txt; do
    [ -f "$src" ] && cp -v "$src" /output/
done
# Wide dragnet for any *.txt trajectory output
find / -name "*pos_log*" -newer /output 2>/dev/null -exec cp -v {} /output/ \; 2>/dev/null || true
ls -la /output/

echo ""
echo "Done."
