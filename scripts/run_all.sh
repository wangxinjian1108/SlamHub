#!/bin/bash
set -euo pipefail

# Usage: ./run_all.sh <recording_dir> [primary_lidar] [output_dir]
#
# Runs the full SlamHub pipeline:
#   01. Convert recording to rosbag
#   02. Run FAST-LIO2 SLAM
#   03. Export SLAM results (global map + trajectory)
#   04. Register secondary LiDARs against primary map
#   05. Solve extrinsic calibration from registration results

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# --- Arguments ---
RECORDING_DIR="${1:?Usage: ./run_all.sh <recording_dir> [primary_lidar] [output_dir]}"
PRIMARY_LIDAR="${2:-remote_front_left_pointcloud}"
OUTPUT_DIR="${3:-output/$(basename "$RECORDING_DIR")}"

# Resolve to absolute paths
RECORDING_DIR="$(cd "$RECORDING_DIR" && pwd)"
mkdir -p "$OUTPUT_DIR"
OUTPUT_DIR="$(cd "$OUTPUT_DIR" && pwd)"

echo "=============================================="
echo " SlamHub Pipeline"
echo "=============================================="
echo " Recording:     $RECORDING_DIR"
echo " Primary LiDAR: $PRIMARY_LIDAR"
echo " Output:        $OUTPUT_DIR"
echo "=============================================="
echo ""

# --- Step 01: Convert to rosbag ---
echo "=============================="
echo " Step 01: Convert to rosbag"
echo "=============================="
ROSBAG_PATH="$OUTPUT_DIR/slam/input.bag"
python "$SCRIPT_DIR/convert_to_rosbag.py" \
    "$RECORDING_DIR" \
    --lidar "$PRIMARY_LIDAR" \
    --output "$ROSBAG_PATH"
echo ""

# --- Step 02: Run SLAM ---
echo "=============================="
echo " Step 02: Run FAST-LIO2 SLAM"
echo "=============================="
python "$SCRIPT_DIR/02_run_slam.py" \
    "$ROSBAG_PATH" \
    --output-dir "$OUTPUT_DIR/slam/"
echo ""

# --- Step 03: Export SLAM results ---
echo "=============================="
echo " Step 03: Export SLAM results"
echo "=============================="
python "$SCRIPT_DIR/03_export_slam_results.py" \
    "$OUTPUT_DIR/slam/" \
    --output-dir "$OUTPUT_DIR/slam/"
echo ""

# --- Step 04: Register secondary LiDARs ---
echo "=============================="
echo " Step 04: Register secondaries"
echo "=============================="
RAW_PCD_DIR="$RECORDING_DIR/raw_pointclouds"
INITIAL_GUESS="$RECORDING_DIR/application.yaml"

INITIAL_GUESS_ARG=""
if [ -f "$INITIAL_GUESS" ]; then
    INITIAL_GUESS_ARG="--initial-guess $INITIAL_GUESS"
fi

for LIDAR_DIR in "$RAW_PCD_DIR"/*/; do
    LIDAR_NAME="$(basename "$LIDAR_DIR")"

    # Skip the primary LiDAR
    if [ "$LIDAR_NAME" = "$PRIMARY_LIDAR" ]; then
        continue
    fi

    echo "  Registering: $LIDAR_NAME"
    python "$SCRIPT_DIR/04_register_secondary.py" \
        --primary-map "$OUTPUT_DIR/slam/global_map.pcd" \
        --trajectory "$OUTPUT_DIR/slam/trajectory.txt" \
        --secondary-dir "$LIDAR_DIR" \
        --method icp \
        --mode frame \
        $INITIAL_GUESS_ARG \
        --output-dir "$OUTPUT_DIR/registration/$LIDAR_NAME/"
    echo ""
done

# --- Step 05: Solve extrinsic calibration ---
echo "=============================="
echo " Step 05: Solve extrinsics"
echo "=============================="
python "$SCRIPT_DIR/05_solve_extrinsic.py" \
    --registration-dir "$OUTPUT_DIR/registration/" \
    --primary-lidar "$PRIMARY_LIDAR" \
    --output "$OUTPUT_DIR/calibration/extrinsics.yaml"
echo ""

# --- Step 06: Quality check (advisory) ---
echo "=============================="
echo " Step 06: Quality check"
echo "=============================="
# Don't fail the pipeline on quality warnings — just surface them.
# Use --thresholds <file> to tighten in production.
python "$SCRIPT_DIR/check_quality.py" --run-dir "$OUTPUT_DIR" || \
    echo "  (quality check returned a FAIL — see above)"
echo ""

echo "=============================================="
echo " Pipeline complete!"
echo " Results in: $OUTPUT_DIR"
echo "=============================================="
