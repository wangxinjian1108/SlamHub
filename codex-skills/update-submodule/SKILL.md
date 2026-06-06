---
name: update-submodule
description: Update an existing SlamHub-style submodule to its latest upstream commit, optionally trigger an image rebuild, and commit the submodule pointer change. Use when the user asks to refresh one tracked model repo.
---

# Update Submodule

Use this skill when the user asks to bring one submodule forward to its latest upstream commit.

## Inputs

- Submodule name or full path

## Workflow

1. Resolve the full submodule path from `.gitmodules`.
2. Verify the directory exists.
3. Update with `git submodule update --init --remote thirdparty/<name>`.
4. Inspect `git diff --submodule thirdparty/<name>`.
5. If nothing changed, report that it is already up to date and stop.
6. Ask whether the image should also be rebuilt if that was not already specified.
7. If rebuild is wanted:
   - verify `docker/<name>/Dockerfile` exists
   - verify `.github/workflows/docker-<name>.yml` exists
   - trigger the workflow manually with `gh`, or push a commit that will match the path filters
8. Stage only the submodule pointer change unless other related repo files truly changed.
9. Commit with `feat: update <name> to latest` unless the user asked for different wording.
10. Push and watch CI if a workflow was triggered.
11. If CI fails because the upstream repo now needs Dockerfile changes, fix those and retry up to 3 rounds.

## Guardrails

- Do not silently rebuild if the user wanted a pointer-only refresh
- Do not touch unrelated submodules
- Call out upstream breakage explicitly when the new revision is incompatible with the existing image

## SLAM-specific upstream-breakage patterns

When the upstream commit moves forward, the most common Dockerfile breakages we've seen:

- New ROS message type added to `package.xml` → image needs an extra `ros-${ROS_DISTRO}-<pkg>` apt install
- C++ standard pin moved from `c++11` to `c++14`/`c++17` (or vice versa) → adjust `sed` patch in Dockerfile
- New header path from PCL/Eigen/OpenCV upgrade → may need patch
- New required environment var → update Dockerfile `ENV`
- New Python dep added to `pyproject.toml` (for pure-Python backends like genz-icp/mad-icp) → bump pinned wheel version in Dockerfile

## Optional: post-update benchmark sanity

For backends wired into `scripts/run_all_backends.sh`, ask whether to re-run the multi-sample benchmark to confirm the upstream change didn't regress trajectory accuracy or cross-LiDAR \|Δt\|. If yes, invoke the run-benchmark skill on a representative sample (default ZL11626 baseline).

## Report

Report:

- old and new submodule revisions
- whether an image rebuild was triggered
- final CI status
- (if benchmark re-run) any change in cross-backend agreement vs prior values
- any upstream incompatibilities that still need work
