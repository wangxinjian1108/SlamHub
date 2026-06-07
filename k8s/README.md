# SlamHub K8s Job manifests

Cluster validation manifests for SlamHub backends. Apply with `kubectl apply -f <file>`. They mount your TEEMO recording at `/data`, run the corresponding `scripts/run_<backend>_in_container.sh`, and write `/output` to the configured volume.

## Files

| Manifest | Purpose | Image |
|----------|---------|-------|
| [fast-livo2-job.yaml](fast-livo2-job.yaml) | Run FAST-LIVO2 (LiDAR + IMU + camera VIO) on one TEEMO sample | `ghcr.io/wangxinjian1108/fast-livo2:latest` |

## Before applying

Each manifest has two `# CHANGE:` markers you'll likely need to edit:

1. **`volumes.data.hostPath.path`** — point to wherever TEEMO recording dirs live on your cluster. If the data is on an NFS PVC, swap `hostPath` for `persistentVolumeClaim` referencing that PVC.
2. **`volumes.output`** — currently `emptyDir` (results disappear when Job deletes); for keeping trajectories, swap for a PVC. Set `claimName` to whatever exists in your namespace (typically a `slamhub-output-pvc`).
3. **`imagePullSecrets`** (commented out) — uncomment + create a docker-registry secret if your cluster doesn't have a public image-pull default for `ghcr.io`. To create:
    ```bash
    kubectl create secret docker-registry ghcr-pull-secret \
      --docker-server=ghcr.io \
      --docker-username=wangxinjian1108 \
      --docker-password=$GITHUB_TOKEN \
      -n <your-namespace>
    ```

## Run

```bash
# 1. Apply
kubectl apply -f k8s/fast-livo2-job.yaml -n <namespace>

# 2. Watch logs
kubectl logs -f job/fast-livo2-validate -n <namespace>

# 3. When complete, copy /output back to your local
POD=$(kubectl get pods -n <namespace> -l app=fast-livo2-validate -o name | head -1)
kubectl cp <namespace>/${POD#pod/}:/output ./fast-livo2-results

# 4. Cleanup (auto-cleans 24h after success via ttlSecondsAfterFinished, or manually:
kubectl delete job fast-livo2-validate -n <namespace>
```

## Expected output

When the Job succeeds, `/output/` (or your PVC) contains:

- `<lidar>_livo.bag` — converted ROS bag with synced PCD + IMU + image streams
- `pos_log.txt` (or similar trajectory file from FAST-LIVO2's evo output)
- `*.pcd` — dense map saved on shutdown
- `ros_logs/` — node-level logs for debugging

Pass criteria:
- Smoke check: `/catkin_ws/devel/lib/fast_livo/fastlivo_mapping` exists ✓
- ROS sanity: `rosversion fast_livo` returns OK ✓
- Pipeline: at least one frame processed (check ros logs)
- Trajectory: `pos_log.txt` has 100+ entries spanning the bag duration

## Customizing

The `args:` block in `fast-livo2-job.yaml` supports two env overrides:

- `LIDAR` — LiDAR subdir under `<recording>/raw_pointclouds/` (default `remote_front_left_pointcloud`)
- `CAMERA_FRAME` — TEEMO camera frame name (default `FRAME_CAMERA_TRAFFIC_FRONT`; pinhole, front-facing)

To swap to a fisheye camera (e.g. `FRAME_CAMERA360_FRONT_LEFT`), edit the env block in the manifest.

## Why this is the easiest cluster validation

This Job:

1. Pulls the GHCR image on the cluster's fast network (faster than my local proxy)
2. Runs the same `run_fastlivo2_in_container.sh` we ship in the repo
3. Lets us validate the build + the runtime + the TEEMO data integration in one step

If anything fails, the failure mode is one of:
- Image pull failure → check `imagePullSecrets`
- ROS init failure → see node logs in `/output/ros_logs/`
- Bag conversion failure → see Step 1 logs (camera/lidar/imu mismatch)
- VIO divergence → trajectory will be present but absurd; compare against other backends
