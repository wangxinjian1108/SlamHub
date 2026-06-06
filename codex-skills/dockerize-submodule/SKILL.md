---
name: dockerize-submodule
description: Generate or update the Dockerfile and GitHub Actions image workflow for a SlamHub-style submodule. Use when a SLAM/LiDAR-odometry repo already exists under thirdparty and needs a runnable image and CI publishing pipeline.
---

# Dockerize Submodule

Use this skill when the user asks to dockerize an existing submodule such as `thirdparty/LIO-SAM` or `LIO-SAM`.

## Inputs

- A submodule path or submodule name

## Repo assumptions

- Source already present as a git submodule under `thirdparty/<name>`
- Docker output path: `docker/<name>/Dockerfile`
- Workflow output path: `.github/workflows/docker-<name>.yml`
- Image target: `ghcr.io/wangxinjian1108/<image-name>` (lowercase, dash form)

## Guardrails

- Read `AGENTS.md` first if it exists
- Reuse existing SlamHub Dockerfiles as the primary style reference
- Build must use local submodule contents from repo root context
- **Never `git clone` inside the Dockerfile**. Always `COPY thirdparty/<name>`.

## Discovery

1. Resolve the full submodule path from `.gitmodules` if only a short name was provided.
2. Read README + the most informative dependency file:
   - `package.xml` (ROS) — distro + ros-* deps
   - `CMakeLists.txt` — C++ standard, find_package() requirements
   - `pyproject.toml` / `requirements.txt` — pure-Python wheel info
   - `setup.sh`, `Makefile`, README install instructions — heavy deps
3. Detect:
   - **Project type**: ROS1 C++ / ROS2 C++ / pure-Python pip-installable
   - **ROS distro**: Noetic (ROS1) or Humble (ROS2) by default
   - **Build system**: `catkin_make` (ROS1), `colcon build` (ROS2), `pip install` (Python)
   - **C++ standard pin** (some upstream projects pin C++11 but PCL 1.10+ requires C++14+ — flag for sed patch)
   - **Heavy deps not in apt**: GTSAM 4.x, ceres-solver 2.x, Sophus, etc. (need source-build steps)
   - **Known compat patches needed**: `<opencv/cv.h>` → `<opencv2/opencv.hpp>`, FLANN serialization for `unordered_map`, `click<8.2` for typer-using CLIs

## Templates

### Template A: ROS1 C++ SLAM (FAST_LIO, LIO-SAM, A-LOAM, LeGO-LOAM, etc.)

- `FROM ros:noetic-ros-base-focal`
- `ENV DEBIAN_FRONTEND=noninteractive PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1 PIP_NO_CACHE_DIR=1 ROS_DISTRO=noetic`
- Standard debug toolchain (git, vim, gdb, cmake, ninja, etc. — same fixed list as existing FAST_LIO/LIO_SAM Dockerfiles)
- `locale-gen en_US.UTF-8 zh_CN.UTF-8` then `LANG=en_US.UTF-8`
- ROS apt deps: `pcl-ros tf2-eigen rviz cv-bridge image-transport eigen-conversions rosbag navigation robot-localization` + `libpcl-dev libeigen3-dev libgflags-dev libgoogle-glog-dev libboost-all-dev libtbb-dev libmetis-dev`
- Source-build heavy deps if needed (GTSAM 4.0.3 example in `docker/LIO_SAM/Dockerfile`)
- `mkdir -p /catkin_ws/src && COPY thirdparty/<dependency> /catkin_ws/src/<dependency> && COPY thirdparty/<name> /catkin_ws/src/<name>`
- Apply known sed patches in-line if upstream is incompatible
- `catkin_make -DCMAKE_BUILD_TYPE=Release` then `test -f /catkin_ws/devel/lib/<name>/<binary>`
- `WORKDIR /catkin_ws`, `CMD ["/bin/bash"]`

### Template B: Pure-Python LiDAR odometry (KISS-ICP, GenZ-ICP, MAD-ICP)

- `FROM ubuntu:22.04`
- Same standard debug toolchain + `python3 python3-pip python3-dev libgl1 libgomp1`
- Optional: `libeigen3-dev libomp-dev` if the wheel sdist needs to compile
- `COPY thirdparty/<name> /opt/<name>` (reference only — we don't `pip install /opt/<name>`)
- `pip3 install <pkg>==<version>` from PyPI (with version pin); add `ninja` first if sdist needs it; pin `click<8.2` if CLI uses typer
- Smoke check: `<cli> --version && python3 -c "import <pkg>; print(<pkg>.__version__)"`
- `RUN pip3 install --no-cache-dir scipy pyyaml open3d numpy`
- `WORKDIR /workspace`, `CMD ["/bin/bash"]`

## Workflow requirements

Write `.github/workflows/docker-<name>.yml` (use `.github/workflows/docker-LIO_SAM.yml` as style reference). Single `build-and-push` job on `ubuntu-latest`:

- Triggers: push to `master`, tag `v*.*.*`, PR to `master`, `workflow_dispatch`
- Paths filter: `docker/<name>/**`, `.github/workflows/docker-<name>.yml`
- Steps: `actions/checkout@v4 with submodules: recursive` → free disk space → `setup-buildx-action@v3` → `login-action@v3` to ghcr → `build-push-action@v5` with `context: .`, `file: docker/<name>/Dockerfile`, `cache-to/cache-from: type=gha,mode=max`
- Tags: `ghcr.io/wangxinjian1108/<image-name>:latest` and `ghcr.io/wangxinjian1108/<image-name>:${{ github.sha }}`
- No Harbor, no Docker Hub — SlamHub publishes only to GHCR

## Validation

- Compare with existing SlamHub Dockerfiles (FAST_LIO, LIO_SAM, genz_icp, mad_icp) for consistency
- Image naming is lowercase dash form (FAST_LIO → fast-lio)
- Workflow builds from repo root, not `docker/<name>`
- All required secrets used correctly (only `GITHUB_TOKEN`, no HF_TOKEN since SlamHub doesn't have gated downloads)

## Done criteria

- `docker/<name>/Dockerfile` exists and matches repo conventions
- `.github/workflows/docker-<name>.yml` exists and triggers correctly
- Short report: chosen template (A or B), major dep decisions, any patches applied, the GHCR image tag
