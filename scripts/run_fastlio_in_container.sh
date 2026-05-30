#!/bin/bash
# Orchestrate FAST-LIO2 run inside the GHCR container.
# Mounted layout:
#   /data        -> recording dir (read-only)
#   /workspace   -> SlamHub repo (scripts, config)
#   /output      -> results out
set -e

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

RECORDING=/data
LIDAR=${LIDAR:-remote_front_left_pointcloud}
BAG=/output/${LIDAR}.bag
PCD_DIR=/catkin_ws/src/FAST_LIO/PCD

mkdir -p /output

echo "=== Step 1: Convert PCD+IMU to Velodyne-format rosbag ==="
python3 /workspace/scripts/convert_to_rosbag_velodyne.py "$RECORDING" \
    --lidar "$LIDAR" -o "$BAG" --scan-line 128 --vfov-min -13 --vfov-max 14

echo ""
echo "=== Step 2: Copy AT128P config into FAST_LIO ==="
cp /workspace/config/fastlio_at128p_velodyne.yaml /catkin_ws/src/FAST_LIO/config/at128p.yaml

# Write a headless launch (no rviz)
cat > /catkin_ws/src/FAST_LIO/launch/mapping_at128p.launch <<'EOF'
<launch>
    <rosparam command="load" file="$(find fast_lio)/config/at128p.yaml" />
    <param name="feature_extract_enable" type="bool" value="0"/>
    <param name="point_filter_num" type="int" value="4"/>
    <param name="max_iteration" type="int" value="3" />
    <param name="filter_size_surf" type="double" value="0.5" />
    <param name="filter_size_map" type="double" value="0.5" />
    <param name="cube_side_length" type="double" value="1000" />
    <param name="runtime_pos_log_enable" type="bool" value="1" />
    <node pkg="fast_lio" type="fastlio_mapping" name="laserMapping" output="screen" />
</launch>
EOF

echo ""
echo "=== Step 3: Start roscore ==="
roscore &
ROSCORE_PID=$!
sleep 3

echo ""
echo "=== Step 4: Launch FAST-LIO2 ==="
roslaunch fast_lio mapping_at128p.launch &
SLAM_PID=$!
sleep 5

echo ""
echo "=== Step 5: Play rosbag ==="
BAG_DURATION=$(rosbag info "$BAG" | grep duration | head -1)
echo "Bag info: $BAG_DURATION"
rosbag play --clock "$BAG"

echo ""
echo "=== Step 6: Wait for processing to flush ==="
sleep 10

# FAST-LIO saves the map on node shutdown
echo "Shutting down SLAM node to trigger PCD save..."
rosnode kill /laserMapping 2>/dev/null || true
sleep 8
kill $SLAM_PID 2>/dev/null || true
kill $ROSCORE_PID 2>/dev/null || true
sleep 3

echo ""
echo "=== Step 7: Collect outputs ==="
ls -la "$PCD_DIR"/ 2>/dev/null || echo "No PCD dir"
cp -v "$PCD_DIR"/*.pcd /output/ 2>/dev/null || echo "No PCD files saved"
# Trajectory log
cp -v /catkin_ws/src/FAST_LIO/Log/pos_log.txt /output/ 2>/dev/null || \
  cp -v ~/.ros/pos_log.txt /output/ 2>/dev/null || \
  find / -name "pos_log.txt" -newer /output 2>/dev/null -exec cp -v {} /output/ \; || \
  echo "No pos_log found"

echo ""
echo "=== Done. Output contents: ==="
ls -la /output/
