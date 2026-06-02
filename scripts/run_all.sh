#!/bin/bash
set -euo pipefail

# Usage: ./run_all.sh [--backend {mad_icp|genz_icp|kiss_icp|fast_lio}] <recording_dir> [primary_lidar] [output_dir]
#
# SlamHub end-to-end pipeline:
#   01 — Primary SLAM
#       MAD-ICP (default): matching-data odometry, no IMU, no Docker (§20: best |dt| across samples)
#       GenZ-ICP:          adaptive-weighted voxel ICP, no IMU, no Docker (best ATE on long recordings)
#       KISS-ICP:          native Python, no IMU, no Docker
#       FAST-LIO:          in GHCR container, LiDAR + IMU
#   02 — Cross-LiDAR registration (icp_pl, point-to-plane)
#   03 — Extrinsic aggregation (B2 axis-info-weighted)
#   04 — Trajectory eval vs LIDAR_TO_MAP (ATE + RPE)
#   05 — Quality alarms

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

BACKEND="mad_icp"
while [[ "${1:-}" == --* ]]; do
    case "$1" in
        --backend)
            BACKEND="$2"; shift 2;;
        --backend=*)
            BACKEND="${1#--backend=}"; shift;;
        *)
            echo "Unknown flag: $1" >&2; exit 1;;
    esac
done

RECORDING_DIR="${1:?Usage: ./run_all.sh [--backend kiss_icp|fast_lio] <recording_dir> [primary_lidar] [output_dir]}"
PRIMARY_LIDAR="${2:-remote_front_left_pointcloud}"
OUTPUT_DIR="${3:-output/$(basename "$RECORDING_DIR")_$BACKEND}"

RECORDING_DIR="$(cd "$RECORDING_DIR" && pwd)"
mkdir -p "$OUTPUT_DIR"
OUTPUT_DIR="$(cd "$OUTPUT_DIR" && pwd)"

echo "=============================================="
echo " SlamHub Pipeline"
echo "=============================================="
echo " Backend:       $BACKEND"
echo " Recording:     $RECORDING_DIR"
echo " Primary LiDAR: $PRIMARY_LIDAR"
echo " Output:        $OUTPUT_DIR"
echo "=============================================="
echo ""

# --- Step 01: Primary SLAM ---
echo "=============================="
echo " Step 01: Primary SLAM ($BACKEND)"
echo "=============================="
case "$BACKEND" in
    mad_icp)
        REUSE_FLAG=""
        if [ -d "$OUTPUT_DIR/../$(basename "$RECORDING_DIR")_kiss_icp/cleaned_pcds" ]; then
            REUSE_FLAG="--cleaned-pcd-dir $OUTPUT_DIR/../$(basename "$RECORDING_DIR")_kiss_icp/cleaned_pcds"
        fi
        python3 "$SCRIPT_DIR/run_mad_icp.py" \
            "$RECORDING_DIR" \
            --primary-lidar "$PRIMARY_LIDAR" \
            --output-dir "$OUTPUT_DIR" \
            $REUSE_FLAG
        ;;
    genz_icp)
        # Reuse cleaned PCDs from a previous KISS-ICP run if available, since
        # the NaN-strip step is identical and slow.
        REUSE_FLAG=""
        if [ -d "$OUTPUT_DIR/../$(basename "$RECORDING_DIR")_kiss_icp/cleaned_pcds" ]; then
            REUSE_FLAG="--cleaned-pcd-dir $OUTPUT_DIR/../$(basename "$RECORDING_DIR")_kiss_icp/cleaned_pcds"
        fi
        python3 "$SCRIPT_DIR/run_genz_icp.py" \
            "$RECORDING_DIR" \
            --primary-lidar "$PRIMARY_LIDAR" \
            --output-dir "$OUTPUT_DIR" \
            $REUSE_FLAG
        ;;
    kiss_icp)
        python3 "$SCRIPT_DIR/run_kiss_icp.py" \
            "$RECORDING_DIR" \
            --primary-lidar "$PRIMARY_LIDAR" \
            --output-dir "$OUTPUT_DIR"
        ;;
    fast_lio)
        # Runs in GHCR container; expects /data, /workspace, /output mounts.
        IMAGE="ghcr.io/wangxinjian1108/fast-lio:latest"
        # Container expects PWD-root mount for scripts + recording mount + output mount.
        docker run --rm \
            -v "$RECORDING_DIR":/data:ro \
            -v "$SCRIPT_DIR/..":/workspace:ro \
            -v "$OUTPUT_DIR":/output \
            -e LIDAR="$PRIMARY_LIDAR" \
            "$IMAGE" \
            /bin/bash /workspace/scripts/run_fastlio_in_container.sh
        # FAST-LIO writes pos_log.txt + scans.pcd; convert pos_log → TUM.
        # First lidar timestamp comes from the first PCD file in the recording.
        FIRST_PCD=$(ls "$RECORDING_DIR/raw_pointclouds/$PRIMARY_LIDAR"/*.pcd | head -1)
        FIRST_TS=$(python3 -c "import sys, pathlib; print(int(pathlib.Path(sys.argv[1]).stem)/1e9)" "$FIRST_PCD")
        python3 "$SCRIPT_DIR/poslog_to_tum.py" \
            "$OUTPUT_DIR/pos_log.txt" "$OUTPUT_DIR/trajectory.txt" "$FIRST_TS"
        # Voxel-downsample the global map for ICP target.
        python3 -c "
import open3d as o3d
pcd = o3d.io.read_point_cloud('$OUTPUT_DIR/scans.pcd')
ds = pcd.voxel_down_sample(0.3)
o3d.io.write_point_cloud('$OUTPUT_DIR/scans_voxel0.3.pcd', ds)
print(f'voxel 0.3m: {len(pcd.points):,} → {len(ds.points):,} points')
"
        ;;
    *)
        echo "Unknown backend: $BACKEND" >&2; exit 1;;
esac
echo ""

# --- Step 02: Register secondary LiDARs ---
echo "=============================="
echo " Step 02: Cross-LiDAR registration (icp_pl)"
echo "=============================="
RAW_PCD_DIR="$RECORDING_DIR/raw_pointclouds"
INITIAL_GUESS="$RECORDING_DIR/application.yaml"

INITIAL_GUESS_ARG=""
if [ -f "$INITIAL_GUESS" ]; then
    INITIAL_GUESS_ARG="--initial-guess $INITIAL_GUESS"
fi

for LIDAR_DIR in "$RAW_PCD_DIR"/*/; do
    LIDAR_NAME="$(basename "$LIDAR_DIR")"
    if [ "$LIDAR_NAME" = "$PRIMARY_LIDAR" ]; then
        continue
    fi
    echo "  Registering: $LIDAR_NAME"
    python3 "$SCRIPT_DIR/04_register_secondary.py" \
        --primary-map "$OUTPUT_DIR/scans_voxel0.3.pcd" \
        --trajectory "$OUTPUT_DIR/trajectory.txt" \
        --secondary-dir "$LIDAR_DIR" \
        --method icp_pl --mode frame --submap-radius 50.0 \
        $INITIAL_GUESS_ARG \
        --output-dir "$OUTPUT_DIR/registration/$LIDAR_NAME"
    echo ""
done

# --- Step 03: Solve extrinsic ---
echo "=============================="
echo " Step 03: Extrinsic aggregation (B2 info-weighted)"
echo "=============================="
python3 "$SCRIPT_DIR/extract_extrinsic_from_registration.py" \
    --primary-trajectory "$OUTPUT_DIR/trajectory.txt" \
    --registration-dir "$OUTPUT_DIR/registration" \
    --initial-guess "$INITIAL_GUESS" \
    --info-weighting \
    --output "$OUTPUT_DIR/calibrated_extrinsics.yaml"
echo ""

# --- Step 04: Trajectory eval (if LIDAR_TO_MAP is available) ---
if [ -d "$RECORDING_DIR/LIDAR_TO_MAP" ]; then
    echo "=============================="
    echo " Step 04: Trajectory eval vs LIDAR_TO_MAP"
    echo "=============================="
    SLAM_FRAME=$([ "$BACKEND" = "fast_lio" ] && echo "imu" || echo "imu")
    python3 "$SCRIPT_DIR/compare_with_lidar_to_map.py" \
        --slam-trajectory "$OUTPUT_DIR/trajectory.txt" \
        --reference-dir "$RECORDING_DIR/LIDAR_TO_MAP" \
        --application-yaml "$INITIAL_GUESS" \
        --output-dir "$OUTPUT_DIR" \
        --slam-frame "$SLAM_FRAME"
    echo ""
fi

# --- Step 05: Quality alarms ---
echo "=============================="
echo " Step 05: Quality alarms"
echo "=============================="
python3 "$SCRIPT_DIR/check_quality.py" \
    --run-dir "$OUTPUT_DIR" \
    --registration-dir "$OUTPUT_DIR/registration" || \
    echo "  (quality check returned a FAIL — see above)"
echo ""

echo "=============================================="
echo " Pipeline complete!"
echo " Results in: $OUTPUT_DIR"
echo "=============================================="
