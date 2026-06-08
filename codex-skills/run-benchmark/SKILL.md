---
name: run-benchmark
description: Run the multi-backend SLAM + cross-LiDAR calibration benchmark on one (or all) AT128P recording samples in SlamHub, then aggregate trajectory metrics, cross-backend agreement, and per-cell |Δt| tables. Use when the user wants to validate a new submodule end-to-end or refresh the multi-sample comparison after a backend change.
---

# Run Benchmark

Use this skill when the user wants to run the full SLAM + cross-LiDAR calibration pipeline on a recording and report all the standard metrics — trajectory length / Z drift / ATE-RPE / cross-backend |Δt| / GT-free trustworthiness.

## Inputs

- A recording directory path or short sample name (e.g. `ZL11626`, `ZL10359`)
- Or `all` for the 7 known samples sequentially (long: 4–6 hours)

Optional flags:
- `--backends a,b,c` — subset of backends (default all 6)
- `--skip-existing` — skip backends with a `calibrated_extrinsics.yaml` already present

## Repo assumptions

- Orchestrator: `scripts/run_all_backends.sh <recording-dir> <output-root>` (idempotent)
- Output convention: `output/multi_sample/<sample-name>/<backend>/`
- Six backends, in this canonical order (KISS-ICP first because it generates `cleaned_pcds/` reused by GenZ/MAD/LIO-SAM-hybrid):
  1. KISS-ICP — native pip, pure-Python voxel ICP
  2. FAST-LIO2 — GHCR docker (`ghcr.io/wangxinjian1108/fast-lio:latest`), ESKF + IMU
  3. GenZ-ICP — native pip, adaptive-weighted voxel ICP
  4. MAD-ICP — native pip, matching-data voxel ICP
  5. LIO-SAM — GHCR docker (`ghcr.io/wangxinjian1108/lio-sam:latest`), factor graph + IMU + loop closure
  6. LIO-SAM\* hybrid — LIO-SAM trajectory + raw-PCD restitched map (in-script post-processing, no separate image)
  7. FAST-LIVO2 — GHCR docker (`ghcr.io/wangxinjian1108/fast-livo2:latest`), LiDAR + IMU + camera VIO. Skipped automatically when image not pulled locally; cluster validation via `k8s/fast-livo2-job.yaml`.

## Guardrails

- Read `AGENTS.md` first if it exists
- Do not modify `.gitmodules`, Dockerfiles, or workflows during a benchmark run
- Don't auto-fork / auto-fix backend errors during benchmark — surface them and let the user decide whether to update / patch
- Don't pollute `master` with output artifacts (`output/multi_sample/` should be in `.gitignore`)

## Workflow

1. Resolve recording. If user passed a short name, look up under `/root/code/LargeCalibService/node_data/fixtures/lio/`. Verify presence of:
   - `<dir>/raw_pointclouds/remote_front_left_pointcloud/*.pcd` (primary)
   - `<dir>/raw_pointclouds/{flash_front,flash_rear,remote_front_right}_pointcloud/*.pcd` (3 secondaries)
   - `<dir>/imu.csv` (for IMU backends)
   - `<dir>/application.yaml` (per-vehicle calibration)

2. Pull GHCR images **only if the corresponding backend output is missing**
   (`output/multi_sample/<sample>/<backend>/calibrated_extrinsics.yaml` absent).
   When all 6 backends already have cached results, the orchestrator's
   idempotent skip means no docker invocation is needed — skip the pull
   entirely. Pure-Python backends (kiss-icp, genz-icp, mad-icp) install
   via pip during pipeline anyway.

3. Determine output dir: `output/multi_sample/<sample-name>/`.

4. Invoke `bash scripts/run_all_backends.sh <recording-dir> output/multi_sample/<sample-name>`. The script:
   - Generates per-recording configs via `gen_fastlio_config.py` / `gen_liosam_config.py` (extrinsics differ per vehicle)
   - Runs the 6 backends in order, skipping any backend that already has `trajectory.txt` + `calibrated_extrinsics.yaml`
   - For each backend's voxel-0.3 map: runs B2 cross-LiDAR ICP (3 secondaries × icp_pl frame mode) + axis-info-weighted aggregation

5. Each long-recording sample (60s) takes ~50–90 min; short (~25s) takes ~25–45 min. Use Monitor on `tail -F` to surface key milestones (backend complete, B2 complete, errors).

6. Validate output. For each backend dir verify `trajectory.txt`, `scans_voxel0.3.pcd`, `calibrated_extrinsics.yaml` (3 secondaries each).

7. Aggregate cross-sample tables when 2+ samples are present:
   ```
   python3 scripts/eval_cross_sample.py
   python3 scripts/eval_per_sample_table.py
   python3 scripts/eval_internal_quality.py    # GT-free trustworthiness signal
   python3 scripts/eval_calib_side_by_side.py  # full 6-DoF dump
   ```

8. Optionally generate visualizations:
   ```
   python3 scripts/viz_calib_scatter.py
   python3 scripts/viz_backend_bias.py
   ```

9. Report per backend, in 1–2 sentences each:
   - trajectory length, Z range, Z std
   - mean ICP fitness for cross-LiDAR registration
   - 3 \|Δt\| values vs application.yaml's factory values
   - flag any (sample, secondary) cell where cross-backend std > 0.30 m (untrustworthy data per §21.7)

## Reference

- 7-sample sweep is documented in §20–§22 of
  `docs/reports/2026-05-30-fastlio-at128p-evaluation.md`
- Sample-difficulty heuristic: long recording (≥60s) + multi-direction motion + ≥100m path → high trust. Short (<30s) single-direction → reject. Use cross-backend std as the gating signal, not absolute |Δt|.

## Done criteria

- All 6 backends populated in `output/multi_sample/<sample-name>/`
- B2 cross-LiDAR calibration produced for each backend
- Per-(sample, secondary) cell summary printed
- Any data-limit cells (cross-backend std > 0.30 m) explicitly flagged
- (If multiple samples) cross-sample reproducibility std reported per backend
