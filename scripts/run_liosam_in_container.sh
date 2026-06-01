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
# Separate bag from FAST-LIO's: accel is sign-flipped here to match GTSAM
# PreintegrationU convention (LIO-SAM requires +g·ẑ at rest, Z-up).
BAG=/output/${LIDAR}_liosam.bag

mkdir -p /output

echo "=== Step 1: Convert PCD+IMU → Velodyne-format rosbag (LIO-SAM IMU sign) ==="
if [ -f "$BAG" ]; then
    echo "Bag $BAG already exists — skipping conversion."
else
    python3 /workspace/scripts/convert_to_rosbag_velodyne.py "$RECORDING" \
        --lidar "$LIDAR" -o "$BAG" --scan-line 128 --vfov-min -13 --vfov-max 14 \
        --negate-accel
fi

echo ""
echo "=== Step 2: Copy LIO-SAM config + launch ==="
cp /workspace/config/liosam_at128p.yaml /catkin_ws/src/LIO-SAM/config/at128p.yaml

# Headless launch (no rviz).
# respawn=false: we want the FIRST crash to be visible so we can grab the
# stacktrace, not have the node silently respawn and obscure it.
cat > /catkin_ws/src/LIO-SAM/launch/run_at128p.launch <<'EOF'
<launch>
    <rosparam file="$(find lio_sam)/config/at128p.yaml" command="load" />

    <node pkg="lio_sam" type="lio_sam_imuPreintegration"  name="lio_sam_imuPreintegration"  output="screen"     respawn="false"/>
    <node pkg="lio_sam" type="lio_sam_imageProjection"    name="lio_sam_imageProjection"    output="screen"     respawn="false"/>
    <node pkg="lio_sam" type="lio_sam_featureExtraction"  name="lio_sam_featureExtraction"  output="screen"     respawn="false"/>
    <node pkg="lio_sam" type="lio_sam_mapOptmization"     name="lio_sam_mapOptmization"     output="screen"     respawn="false"/>
</launch>
EOF

echo ""
echo "=== Step 3: Start roscore ==="
roscore &
ROSCORE_PID=$!
sleep 3

echo ""
echo "=== Step 4: Launch LIO-SAM ==="
# Capture each node's stdout+stderr to its own log file so we can read the
# crash backtrace even after the process dies. roslaunch's per-node log files
# (/root/.ros/log/.../*.log) only contain rosout-routed messages, not the
# stderr that prints the SIGABRT cause.
mkdir -p /output/node_logs
ulimit -c unlimited
# Start nodes individually (instead of roslaunch) so we control redirection.
# Load params at ROOT (yaml has its own `lio_sam:` top-level key — namespacing
# again would create /lio_sam/lio_sam/* and the nodes would not find them).
rosparam load /catkin_ws/src/LIO-SAM/config/at128p.yaml
/catkin_ws/devel/lib/lio_sam/lio_sam_imuPreintegration \
    __name:=lio_sam_imuPreintegration > /output/node_logs/imuPreintegration.log 2>&1 &
IMU_PID=$!
/catkin_ws/devel/lib/lio_sam/lio_sam_imageProjection \
    __name:=lio_sam_imageProjection > /output/node_logs/imageProjection.log 2>&1 &
IP_PID=$!
/catkin_ws/devel/lib/lio_sam/lio_sam_featureExtraction \
    __name:=lio_sam_featureExtraction > /output/node_logs/featureExtraction.log 2>&1 &
FE_PID=$!
/catkin_ws/devel/lib/lio_sam/lio_sam_mapOptmization \
    __name:=lio_sam_mapOptmization > /output/node_logs/mapOptmization.log 2>&1 &
MO_PID=$!
sleep 5

# Sanity: confirm mapOptmization didn't die immediately
if ! kill -0 "$MO_PID" 2>/dev/null; then
    echo "!!! lio_sam_mapOptmization died at startup. Tail of its log:"
    tail -60 /output/node_logs/mapOptmization.log
fi

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

# Diagnostic: log the message count of every LIO-SAM topic so we can identify
# where the data stops flowing. Runs in background, captures 65 s of stats.
(
    sleep 5  # wait for processing to settle
    echo "=== Topic message rates (rostopic hz, 25 sample window) ==="
    for t in /velodyne_points /imu/data \
             /lio_sam/deskew/cloud_info \
             /lio_sam/feature/cloud_info \
             /lio_sam/mapping/odometry \
             /lio_sam/mapping/path \
             /lio_sam/mapping/cloud_registered_raw; do
        echo "--- $t ---"
        timeout 6 rostopic hz "$t" -w 25 2>&1 | head -5
    done
) > /output/topic_rates.log 2>&1 &
RATES_PID=$!

rosbag play --clock "$BAG"

echo ""
echo "=== Step 6: Wait for processing to flush ==="
sleep 15

# Save current map. NOTE: saveMapService prepends $HOME to destination, so
# `destination: '/output/'` → `/root/output/` inside the container, which is
# NOT our mounted /output. Use `/../output/` so $HOME + dest = /root/../output
# = /output. (See thirdparty/LIO-SAM/src/mapOptmization.cpp:saveMapService.)
echo "Asking LIO-SAM to save map..."
rosservice call /lio_sam/save_map "resolution: 0.0
destination: '/../output/'" 2>&1 || echo "(service call failed, will rely on shutdown save)"
sleep 5

echo ""
echo "=== Step 7: Shut down ==="
kill $REC_PID 2>/dev/null || true
sleep 2
rosnode kill --all 2>/dev/null || true
sleep 5
kill $TF_PID 2>/dev/null || true
kill $IMU_PID $IP_PID $FE_PID $MO_PID 2>/dev/null || true
kill $ROSCORE_PID 2>/dev/null || true
sleep 2

echo ""
echo "=== Step 8: Output summary ==="
# Copy out ROS logs for debugging the imageProjection / mapOptimization nodes.
mkdir -p /output/ros_logs
cp -rL /root/.ros/log/latest/. /output/ros_logs/ 2>/dev/null || true

echo "--- Final tail of each node log (last 40 lines) ---"
# Node logs may have been wiped by saveMapService's `rm -r /output/`.
shopt -s nullglob
for f in /output/node_logs/*.log; do
    echo ""
    echo "===== $f ====="
    tail -40 "$f"
done
shopt -u nullglob
ls -la /output/
echo ""
echo "Done."
