#!/bin/bash
# Orchestrate LIO-SAM in the GHCR image.
# Mounts:
#   /data       — recording dir (read-only)
#   /workspace  — SlamHub repo (scripts, config)
#   /output     — results out
set -e
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

RECORDING=/data
LIDAR=${LIDAR:-remote_front_left_pointcloud}
BAG=/output/${LIDAR}.bag

mkdir -p /output

echo "=== Step 1: Convert PCD+IMU → Velodyne-format rosbag ==="
python3 /workspace/scripts/convert_to_rosbag_velodyne.py "$RECORDING" \
    --lidar "$LIDAR" -o "$BAG" --scan-line 128 --vfov-min -13 --vfov-max 14

echo ""
echo "=== Step 2: Copy LIO-SAM config + launch ==="
cp /workspace/config/liosam_at128p.yaml /catkin_ws/src/LIO-SAM/config/at128p.yaml

# Headless launch (no rviz)
cat > /catkin_ws/src/LIO-SAM/launch/run_at128p.launch <<'EOF'
<launch>
    <rosparam file="$(find lio_sam)/config/at128p.yaml" command="load" />

    <node pkg="lio_sam" type="lio_sam_imuPreintegration"  name="lio_sam_imuPreintegration"  output="screen"     respawn="true"/>
    <node pkg="lio_sam" type="lio_sam_imageProjection"    name="lio_sam_imageProjection"    output="screen"     respawn="true"/>
    <node pkg="lio_sam" type="lio_sam_featureExtraction"  name="lio_sam_featureExtraction"  output="screen"     respawn="true"/>
    <node pkg="lio_sam" type="lio_sam_mapOptmization"     name="lio_sam_mapOptmization"     output="screen"     respawn="true"/>
</launch>
EOF

echo ""
echo "=== Step 3: Start roscore ==="
roscore &
ROSCORE_PID=$!
sleep 3

echo ""
echo "=== Step 4: Launch LIO-SAM ==="
roslaunch lio_sam run_at128p.launch &
SLAM_PID=$!
sleep 5

echo ""
echo "=== Step 5: Play rosbag ==="
BAG_DURATION=$(rosbag info "$BAG" | grep duration | head -1)
echo "Bag info: $BAG_DURATION"

# LIO-SAM also subscribes to a TF link tree; provide a static identity transform
# from base_link → lidar_link so PointCloud2 is in a known frame.
rosrun tf static_transform_publisher 0 0 0 0 0 0 base_link lidar_link 100 &
TF_PID=$!

# Record the path topic to a bag for trajectory extraction.
rosbag record -O /output/lio_sam_path.bag /lio_sam/mapping/path /lio_sam/mapping/odometry &
REC_PID=$!
sleep 1

rosbag play --clock "$BAG"

echo ""
echo "=== Step 6: Wait for processing to flush ==="
sleep 15

# Save current map (LIO-SAM saves on shutdown if savePCD: true)
echo "Asking LIO-SAM to save map..."
rosservice call /lio_sam/save_map "resolution: 0.0
destination: '/output/'" 2>&1 || echo "(service call failed, will rely on shutdown save)"
sleep 5

echo ""
echo "=== Step 7: Shut down ==="
kill $REC_PID 2>/dev/null || true
sleep 2
rosnode kill --all 2>/dev/null || true
sleep 5
kill $TF_PID 2>/dev/null || true
kill $SLAM_PID 2>/dev/null || true
kill $ROSCORE_PID 2>/dev/null || true
sleep 2

echo ""
echo "=== Step 8: Output summary ==="
ls -la /output/
echo ""
echo "Done."
