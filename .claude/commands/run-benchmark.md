Run the multi-backend SLAM + cross-LiDAR calibration benchmark on one recording.

## Argument
`$ARGUMENTS` is one of:
- A recording directory path (e.g. `/root/code/LargeCalibService/node_data/fixtures/lio/ZL11626_*`)
- A short sample name (e.g. `ZL11626`, `ZL10359`) — looked up under the default fixture root
- `all` — run all 7 known samples sequentially (long: 4-6 hours total)

Optional flags after the recording:
- `--backends a,b,c` — subset of backends to run (default: all 6)
- `--skip-existing` — skip backends that already have `output/multi_sample/<sample>/<backend>/calibrated_extrinsics.yaml`

## Steps

1. **Resolve recording** — if `$ARGUMENTS` is a short sample name, look it up under
   `/root/code/LargeCalibService/node_data/fixtures/lio/`. Verify:
   - `<dir>/raw_pointclouds/remote_front_left_pointcloud/*.pcd` exists (primary LiDAR)
   - `<dir>/raw_pointclouds/{flash_front,flash_rear,remote_front_right}_pointcloud/*.pcd` (secondaries for cross-LiDAR)
   - `<dir>/imu.csv` (for IMU-using backends: FAST-LIO, LIO-SAM)
   - `<dir>/application.yaml` (per-vehicle calibration)
   If any is missing, report and stop.

2. **Pre-flight: docker images cached?** — run:
   ```
   docker images | grep -E "fast-lio|lio-sam|genz-icp|mad-icp"
   ```
   For any backend image that's not present, pull it:
   ```
   docker pull ghcr.io/wangxinjian1108/<image>:latest
   ```
   Skip pull on PR / CI.

3. **Determine output dir** — `output/multi_sample/<sample-name>/`. Create if absent.
   `<sample-name>` = first 7 chars of the directory basename (e.g. `ZL11626`).

4. **Invoke the orchestrator** — the canonical entry point is
   `scripts/run_all_backends.sh <recording-dir> <output-root>`. It is idempotent
   (skips backends that already have `trajectory.txt` + `calibrated_extrinsics.yaml`).
   Order:
   1. KISS-ICP (always first — it generates `cleaned_pcds/` reused by GenZ/MAD/LIO-SAM-hybrid)
   2. FAST-LIO2 (GHCR docker, generates per-recording config via `gen_fastlio_config.py`)
   3. GenZ-ICP (native pip, reuses cleaned_pcds)
   4. MAD-ICP (native pip, reuses cleaned_pcds)
   5. LIO-SAM (GHCR docker, per-recording config via `gen_liosam_config.py`)
   6. LIO-SAM\* hybrid (LIO-SAM trajectory + raw-PCD restitched map)
   Then for each backend's voxel-0.3 map: B2 cross-LiDAR registration (3 secondaries × icp_pl frame mode) + axis-info-weighted aggregation.

5. **Watch progress** — the orchestrator prints `=== <step> ===` headers. Use Monitor on `tail -F` of the log to surface key events: backend completion, B2 registration completion, errors. Each long-recording sample (60s) takes ~50-90 min; short (~25s) takes ~25-45 min.

6. **Validate output** — when the orchestrator completes, for each backend dir verify:
   - `trajectory.txt` exists and is non-empty (>1 line)
   - `scans_voxel0.3.pcd` exists
   - `calibrated_extrinsics.yaml` exists with all 3 secondaries

7. **Aggregate cross-sample / per-cell tables** (if more than one sample's data is present):
   ```
   python3 scripts/eval_cross_sample.py
   python3 scripts/eval_per_sample_table.py
   python3 scripts/eval_internal_quality.py    # GT-free trustworthiness signal (§21.4)
   python3 scripts/eval_calib_side_by_side.py  # full 6-DoF dump
   ```

8. **Visualize (optional)** — for the last-run sample:
   ```
   python3 scripts/viz_calib_scatter.py
   python3 scripts/viz_backend_bias.py
   ```

9. **Report** — print one or two sentences per backend:
   - trajectory length, Z range, Z std (from `trajectory.txt`)
   - mean ICP fitness for cross-LiDAR registration
   - 3 \|Δt\| values vs application.yaml's factory values
   - flag any cell with cross-backend std > 0.30 m (untrustworthy data, §21.7)

## Reference: 7 known sample sample profiles (from §21)

| Sample | Frames | Time (s) | Path (m) | Speed (m/s) | Difficulty |
|--------|-------:|---------:|---------:|------------:|------------|
| ZL11626 | 600 | 60 | 456 | 7.6 | baseline (long+fast) |
| ZL10359 | 256 | 26 | 26  | 1.0 | hard (short+slow+single-direction) |
| ZL10966 | 209 | 21 | 42  | 2.0 | medium |
| ZL10968 | 290 | 29 | 152 | 5.3 | well-conditioned |
| ZL11881 | 320 | 32 | 81  | 2.5 | medium |
| ZL12332 | 607 | 61 | 83  | 1.4 | best cross-backend agreement |
| ZL12382 | 602 | 60 | 25  | 0.4 | hard (near-stationary) |

ZL10359 / ZL12382 expected to produce cross-backend std > 0.3 m on most secondaries — that's the data limit, not a backend regression.
