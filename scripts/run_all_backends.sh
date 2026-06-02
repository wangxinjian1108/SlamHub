#!/bin/bash
# Run all 6 SLAM backends + B2 cross-LiDAR calibration on one recording.
#
# Usage: run_all_backends.sh <recording-dir> <output-root>
#
# Produces:
#   <output-root>/{ghcr_run_v3,kiss_icp_run,liosam_run,liosam_run_hybrid,
#                  genz_icp_run,mad_icp_run}/
#       trajectory.txt, scans*.pcd, registration/, calibrated_extrinsics.yaml
#
# The 6 backends:
#   1. FAST-LIO2  — IMU + LiDAR, ESKF, GHCR docker image
#   2. KISS-ICP   — pure-LiDAR voxel ICP, native pip
#   3. LIO-SAM    — IMU + LiDAR, factor graph + loop closure, GHCR docker image
#   4. LIO-SAM*   — LIO-SAM trajectory + raw-PCD restitched map (hybrid)
#   5. GenZ-ICP   — pure-LiDAR adaptive-weighted voxel ICP, native pip
#   6. MAD-ICP    — pure-LiDAR matching-data odometry, native pip
#
# Skips a backend if its output dir already exists & has trajectory.txt.

set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

REC="${1:?usage: $0 <recording-dir> <output-root>}"
OUT="${2:?usage: $0 <recording-dir> <output-root>}"
LIDAR="${3:-remote_front_left_pointcloud}"
REC=$(cd "$REC" && pwd)
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)
NAME=$(basename "$REC")

echo "=============================================="
echo " SlamHub multi-backend benchmark"
echo "=============================================="
echo " sample : $NAME"
echo " primary: $LIDAR"
echo " out    : $OUT"
echo "=============================================="

# Skip helper — return 0 if the backend's trajectory.txt already exists.
have_traj() {
    local d="$1"
    [ -f "$d/trajectory.txt" ] && [ -s "$d/trajectory.txt" ]
}

run_step() {
    local label="$1"
    shift
    echo ""
    echo "=== $label ==="
    "$@"
    local rc=$?
    if [ $rc -ne 0 ]; then
        echo "!!! $label failed (exit $rc) — continuing"
    fi
    return $rc
}

# === 1. KISS-ICP (always run first — its cleaned_pcds/ is reused below) ===
KISS_DIR="$OUT/kiss_icp_run"
if have_traj "$KISS_DIR"; then
    echo "--- skip KISS-ICP (already done at $KISS_DIR)"
else
    run_step "KISS-ICP" python3 "$SCRIPT_DIR/run_kiss_icp.py" \
        "$REC" --primary-lidar "$LIDAR" --output-dir "$KISS_DIR"
fi
CLEAN="$KISS_DIR/cleaned_pcds"

# === 2. FAST-LIO2 (in GHCR image) ===
FL_DIR="$OUT/ghcr_run_v3"
if have_traj "$FL_DIR"; then
    echo "--- skip FAST-LIO (already done)"
else
    mkdir -p "$FL_DIR"
    # Generate per-recording FAST-LIO config (extrinsic_T/R per vehicle)
    FL_CFG="$FL_DIR/fastlio_config.yaml"
    python3 "$SCRIPT_DIR/gen_fastlio_config.py" \
        "$REC" "$FL_CFG" --lidar "$LIDAR" || true
    docker run --rm \
        -v "$REC":/data:ro \
        -v "$(cd "$SCRIPT_DIR/.." && pwd)":/workspace:ro \
        -v "$FL_DIR":/output \
        -e LIDAR="$LIDAR" \
        -e FASTLIO_CONFIG=/output/fastlio_config.yaml \
        ghcr.io/wangxinjian1108/fast-lio:latest \
        /bin/bash /workspace/scripts/run_fastlio_in_container.sh
    # Convert pos_log → TUM
    FIRST_PCD=$(ls "$REC/raw_pointclouds/$LIDAR"/*.pcd | head -1)
    FIRST_TS=$(python3 -c "import sys, pathlib; print(int(pathlib.Path(sys.argv[1]).stem)/1e9)" "$FIRST_PCD")
    python3 "$SCRIPT_DIR/poslog_to_tum.py" \
        "$FL_DIR/pos_log.txt" "$FL_DIR/trajectory.txt" "$FIRST_TS" || true
    # Voxel target for B2
    python3 -c "
import open3d as o3d, glob
import os
pcds = glob.glob('$FL_DIR/scans*.pcd')
src = next((p for p in pcds if 'voxel' not in p), None)
if src is None:
    print('no scans.pcd found')
else:
    pcd = o3d.io.read_point_cloud(src)
    ds = pcd.voxel_down_sample(0.3)
    o3d.io.write_point_cloud('$FL_DIR/scans_voxel0.3.pcd', ds)
    print(f'voxel 0.3m: {len(pcd.points):,} → {len(ds.points):,} points')
" || true
fi

# === 3. GenZ-ICP ===
GENZ_DIR="$OUT/genz_icp_run"
if have_traj "$GENZ_DIR"; then
    echo "--- skip GenZ-ICP (already done)"
else
    run_step "GenZ-ICP" python3 "$SCRIPT_DIR/run_genz_icp.py" \
        "$REC" --primary-lidar "$LIDAR" --output-dir "$GENZ_DIR" \
        --cleaned-pcd-dir "$CLEAN"
fi

# === 4. MAD-ICP ===
MAD_DIR="$OUT/mad_icp_run"
if have_traj "$MAD_DIR"; then
    echo "--- skip MAD-ICP (already done)"
else
    run_step "MAD-ICP" python3 "$SCRIPT_DIR/run_mad_icp.py" \
        "$REC" --primary-lidar "$LIDAR" --output-dir "$MAD_DIR" \
        --cleaned-pcd-dir "$CLEAN"
fi

# === 5. LIO-SAM (GHCR docker image; converts its own bag) ===
LS_DIR="$OUT/liosam_run"
if have_traj "$LS_DIR"; then
    echo "--- skip LIO-SAM (already done)"
else
    mkdir -p "$LS_DIR"
    # Generate a per-recording LIO-SAM config (extrinsicTrans / extrinsicRot
    # pinned to this vehicle's calibration). Without this, LIO-SAM uses the
    # ZL11626-tuned defaults and diverges on other recordings.
    LIOSAM_CFG="$LS_DIR/liosam_config.yaml"
    python3 "$SCRIPT_DIR/gen_liosam_config.py" \
        "$REC" "$LIOSAM_CFG" --lidar "$LIDAR" || true
    docker run --rm \
        -v "$REC":/data:ro \
        -v "$(cd "$SCRIPT_DIR/.." && pwd)":/workspace:ro \
        -v "$LS_DIR":/output \
        -e LIDAR="$LIDAR" \
        -e LIOSAM_CONFIG=/output/liosam_config.yaml \
        ghcr.io/wangxinjian1108/lio-sam:latest \
        /bin/bash /workspace/scripts/run_liosam_in_container.sh || true
    # Post-process: extract trajectory from transformations.pcd, prep for B2
    if [ -f "$LS_DIR/transformations.pcd" ]; then
        python3 "$SCRIPT_DIR/extract_liosam_trajectory.py" \
            "$LS_DIR/transformations.pcd" "$LS_DIR/trajectory_raw.txt" || true
        python3 "$SCRIPT_DIR/prep_liosam_for_b2.py" \
            --liosam-dir "$LS_DIR" --recording "$REC" \
            --primary-lidar "$LIDAR" || true
    fi
fi

# === 6. LIO-SAM* hybrid (LIO-SAM trajectory + raw-PCD restitched map) ===
LSH_DIR="$OUT/liosam_run_hybrid"
if have_traj "$LSH_DIR"; then
    echo "--- skip LIO-SAM* hybrid (already done)"
elif [ -f "$LS_DIR/trajectory_lidar.txt" ] && [ -d "$CLEAN" ]; then
    mkdir -p "$LSH_DIR"
    cp "$LS_DIR/trajectory.txt" "$LSH_DIR/trajectory.txt"
    run_step "LIO-SAM* hybrid stitch" python3 "$SCRIPT_DIR/stitch_liosam_raw_map.py" \
        --cleaned-pcd-dir "$CLEAN" \
        --trajectory-lidar "$LS_DIR/trajectory_lidar.txt" \
        --output-dir "$LSH_DIR"
fi

# === B2 cross-LiDAR registration + aggregation, for each backend that has a map ===
SECONDARIES=(flash_front_pointcloud flash_rear_pointcloud remote_front_right_pointcloud)

run_b2_for() {
    local bdir="$1"
    local bname="$(basename "$bdir")"
    if [ ! -f "$bdir/trajectory.txt" ] || [ ! -f "$bdir/scans_voxel0.3.pcd" ]; then
        echo "--- skip B2 for $bname (missing trajectory.txt or scans_voxel0.3.pcd)"
        return 0
    fi
    if [ -f "$bdir/calibrated_extrinsics.yaml" ]; then
        echo "--- skip B2 for $bname (already done)"
        return 0
    fi
    mkdir -p "$bdir/registration"
    for SEC in "${SECONDARIES[@]}"; do
        local SECDIR="$bdir/registration/$SEC"
        if [ -f "$SECDIR/summary.yaml" ]; then continue; fi
        echo "  [B2 / $bname] register $SEC"
        python3 "$SCRIPT_DIR/04_register_secondary.py" \
            --primary-map "$bdir/scans_voxel0.3.pcd" \
            --trajectory "$bdir/trajectory.txt" \
            --secondary-dir "$REC/raw_pointclouds/$SEC" \
            --initial-guess "$REC/application.yaml" \
            --method icp_pl --mode frame --submap-radius 50.0 \
            --output-dir "$SECDIR" 2>&1 | tail -2
    done
    python3 "$SCRIPT_DIR/extract_extrinsic_from_registration.py" \
        --primary-trajectory "$bdir/trajectory.txt" \
        --registration-dir "$bdir/registration" \
        --initial-guess "$REC/application.yaml" \
        --info-weighting \
        --output "$bdir/calibrated_extrinsics.yaml" 2>&1 | tail -3
}

echo ""
echo "=============================================="
echo " B2 cross-LiDAR registration + aggregation"
echo "=============================================="
for bdir in "$KISS_DIR" "$FL_DIR" "$GENZ_DIR" "$MAD_DIR" "$LS_DIR" "$LSH_DIR"; do
    run_b2_for "$bdir"
done

echo ""
echo "=============================================="
echo " Done: $NAME"
echo " Results in: $OUT"
echo "=============================================="
