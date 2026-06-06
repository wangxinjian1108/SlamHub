Dockerize a SLAM submodule: generate a minimal Dockerfile and GitHub Actions workflow for it.

## Argument
`$ARGUMENTS` is the submodule path or name (e.g. `thirdparty/LIO-SAM` or just `LIO-SAM`).

## Steps

1. **Locate the submodule** — search `.gitmodules` to resolve the full path if only a name was given. Read the submodule directory to understand the project type.

2. **Detect project type** — SlamHub submodules typically fall into one of three buckets. Inspect `pyproject.toml`, `package.xml`, `CMakeLists.txt`, `requirements.txt`, README, install scripts to decide:

   - **ROS1 C++ SLAM** (FAST_LIO, LIO-SAM, A-LOAM, LeGO-LOAM, LIO-LIVOX, etc.): has `package.xml`, depends on ROS Noetic, built with `catkin_make`. Base image: `ros:noetic-ros-base-focal`.
   - **ROS2 C++ SLAM** (DLIO, LIO-SAM-ROS2, etc.): `package.xml` with `<build_depend>ament_*`, built with `colcon`. Base image: `ros:humble-ros-base-jammy`.
   - **Pure-Python LiDAR odometry** (KISS-ICP, GenZ-ICP, MAD-ICP): pip-installable wheel + `pyproject.toml` exposing `[project.scripts]`. Base image: `ubuntu:22.04` + `pip install <pkg>`.

   Detect:
   - **ROS distro**: `sed -n '/<build_depend>/p' package.xml`, or scan READMEs for `noetic`/`humble`/`foxy`. Default ROS1 → noetic, ROS2 → humble.
   - **Build system**: `catkin_make` (ROS1) vs `colcon build` (ROS2) vs `pip install` (pure Python). Read README and CI files for the documented build invocation.
   - **C++ standard**: scan `CMakeLists.txt` for `CMAKE_CXX_STANDARD` or `-std=c++NN`. Some upstream projects pin C++11 but require C++14+ (PCL 1.10, GTSAM 4.0+). Plan a `sed` patch in the Dockerfile if you spot this.
   - **Heavy deps not in apt**: GTSAM (LIO-SAM), Sophus (some LIO variants), ceres-solver (LeGO-LOAM, A-LOAM 2). These need source-build steps.
   - **Python wheel**: read `pyproject.toml` for `[project.scripts]`. The CLI entry point goes into the smoke check at the end of the Dockerfile.

3. **Create the Dockerfile** — write `docker/<name>/Dockerfile` (name is the last path segment of the submodule, create the directory if it doesn't exist). Pick the right template:

   ### Template A: ROS1 C++ SLAM (FAST_LIO, LIO-SAM, etc.)

   **a. Base image & ENV**
   ```dockerfile
   FROM ros:noetic-ros-base-focal

   ENV DEBIAN_FRONTEND=noninteractive \
       PYTHONDONTWRITEBYTECODE=1 \
       PYTHONUNBUFFERED=1 \
       PIP_NO_CACHE_DIR=1 \
       ROS_DISTRO=noetic
   ```

   **b. Debug-friendly toolchain (matches existing FAST_LIO / LIO_SAM Dockerfiles)**
   ```dockerfile
   RUN apt-get update && apt-get install -y --no-install-recommends \
       git zsh vim git-lfs wget unzip bzip2 ca-certificates \
       openssh-server clang-format htop iotop rsync ffmpeg curl \
       cmake make less time sqlite3 tree gdb g++ ninja-build \
       build-essential tmux locales lsb-release nano nethogs \
       net-tools valgrind xz-utils sudo pciutils \
       && rm -rf /var/lib/apt/lists/*

   RUN locale-gen en_US.UTF-8 zh_CN.UTF-8 && update-locale LANG=en_US.UTF-8
   ENV LANG=en_US.UTF-8 LANGUAGE=en_US:en LC_ALL=en_US.UTF-8
   ```

   **c. ROS + SLAM-specific apt packages** (PCL, Eigen, tf2-eigen, navigation, robot_localization for IMU-based stacks)
   ```dockerfile
   RUN apt-get update && apt-get install -y --no-install-recommends \
       ros-${ROS_DISTRO}-pcl-ros ros-${ROS_DISTRO}-tf2-eigen \
       ros-${ROS_DISTRO}-rviz ros-${ROS_DISTRO}-cv-bridge \
       ros-${ROS_DISTRO}-image-transport ros-${ROS_DISTRO}-eigen-conversions \
       ros-${ROS_DISTRO}-rosbag ros-${ROS_DISTRO}-navigation \
       ros-${ROS_DISTRO}-robot-localization \
       python3-catkin-tools python3-pip python3-rosbag \
       libpcl-dev libeigen3-dev libgflags-dev libgoogle-glog-dev \
       libboost-all-dev libtbb-dev libmetis-dev \
       && rm -rf /var/lib/apt/lists/*

   RUN mkdir -p /var/run/sshd /root/.ssh && ssh-keygen -A
   RUN pip3 install --no-cache-dir scipy
   ```

   **d. Optional: build heavy deps from source** — if the submodule needs GTSAM, Sophus, ceres, etc. that aren't in apt at the right version, add source-build `RUN` blocks here. Example for GTSAM 4.0.3 (used by LIO-SAM):
   ```dockerfile
   RUN git clone --branch 4.0.3 --depth 1 https://github.com/borglab/gtsam.git /tmp/gtsam && \
       cd /tmp/gtsam && mkdir build && cd build && \
       cmake -DCMAKE_BUILD_TYPE=Release -DGTSAM_BUILD_TESTS=OFF \
             -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
             -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ROT3_EXPMAP=ON .. && \
       make -j$(nproc) && make install && \
       rm -rf /tmp/gtsam && ldconfig
   ```

   **e. Bring in source via COPY (NOT in-image git clone)** — this is the SlamHub convention. CI does `actions/checkout@v4 with submodules: recursive`, which makes all submodule trees available in the build context.
   ```dockerfile
   RUN mkdir -p /catkin_ws/src
   COPY thirdparty/<dependency-submodule> /catkin_ws/src/<dependency-submodule>
   COPY thirdparty/<name> /catkin_ws/src/<name>
   ```
   Reasons we COPY instead of clone:
   - Reproducibility: we pin a specific commit via the SlamHub gitlink, not "whatever HEAD is today"
   - Network: avoids needing internet inside the build for code fetch
   - Patching: any sed patches downstream apply to the pinned commit

   **f. Apply patches if needed** — common cases:
   - C++11 → C++17 (PCL 1.10 + GTSAM 4 require C++14+). Use a permissive sed:
     ```dockerfile
     RUN sed -i -E 's|-std=c\+\+1[14]|-std=c++17|g' /catkin_ws/src/<name>/CMakeLists.txt
     ```
   - Legacy OpenCV 2 header `<opencv/cv.h>` → OpenCV 4 path `<opencv2/opencv.hpp>`:
     ```dockerfile
     RUN sed -i 's|#include <opencv/cv.h>|#include <opencv2/opencv.hpp>|g' \
         /catkin_ws/src/<name>/include/utility.h
     ```
   - FLANN 1.9 missing `Serializer<std::unordered_map>` (PCL 1.10 needs it) — add the specialization via Python script. See `docker/LIO_SAM/Dockerfile` for the canonical patch.

   **g. catkin_make build**
   ```dockerfile
   RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
       cd /catkin_ws && \
       catkin_make -j\$(nproc) -DCMAKE_BUILD_TYPE=Release" && \
       test -f /catkin_ws/devel/lib/<name>/<entry_executable> && \
       echo "<name> built successfully"

   RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc && \
       echo "source /catkin_ws/devel/setup.bash" >> /etc/bash.bashrc

   WORKDIR /catkin_ws
   CMD ["/bin/bash"]
   ```

   ### Template B: Pure-Python LiDAR odometry (KISS-ICP, GenZ-ICP, MAD-ICP, etc.)

   **a. Base image** — thin Ubuntu 22.04
   ```dockerfile
   FROM ubuntu:22.04

   ENV DEBIAN_FRONTEND=noninteractive \
       PYTHONDONTWRITEBYTECODE=1 \
       PYTHONUNBUFFERED=1 \
       PIP_NO_CACHE_DIR=1
   ```

   **b. Same debug-friendly toolchain** (matches genz_icp / mad_icp Dockerfiles):
   ```dockerfile
   RUN apt-get update && apt-get install -y --no-install-recommends \
       git zsh vim git-lfs wget unzip bzip2 ca-certificates \
       openssh-server clang-format htop iotop rsync ffmpeg curl \
       cmake make less time sqlite3 tree gdb g++ ninja-build \
       build-essential tmux locales lsb-release nano nethogs \
       net-tools valgrind xz-utils sudo pciutils \
       python3 python3-pip python3-dev python3-venv \
       libgl1 libgomp1 \
       && rm -rf /var/lib/apt/lists/*

   RUN locale-gen en_US.UTF-8 zh_CN.UTF-8 && update-locale LANG=en_US.UTF-8
   ENV LANG=en_US.UTF-8 LANGUAGE=en_US:en LC_ALL=en_US.UTF-8
   RUN mkdir -p /var/run/sshd /root/.ssh && ssh-keygen -A
   ```

   **c. C++/native build deps (only if the wheel sdist needs to compile)** — Eigen / OpenMP / pybind11 are common. Check the `pyproject.toml` build-system:
   ```dockerfile
   RUN apt-get update && apt-get install -y --no-install-recommends \
       libeigen3-dev libomp-dev \
       && rm -rf /var/lib/apt/lists/*
   ```

   **d. COPY the source for reproducibility, even though we install from PyPI**
   ```dockerfile
   COPY thirdparty/<name> /opt/<name>
   ```
   We don't `pip install /opt/<name>`. The wheel from PyPI is what we install. The source is here so a reader can audit which commit matches the wheel.

   **e. pip install + sanity check**
   ```dockerfile
   RUN pip3 install --no-cache-dir ninja  # if sdist needs ninja
   RUN pip3 install --no-cache-dir <pkg-name>==<version> [pinned constraints e.g. "click<8.2"]
   RUN <cli-name> --version && python3 -c "import <pkg>; print('<pkg>', <pkg>.__version__)"
   ```
   If the package has known typer/click compat issues (MAD-ICP), pin `click<8.2`. Document why in a comment.

   **f. Useful runtime extras for SlamHub downstream pipeline**
   ```dockerfile
   RUN pip3 install --no-cache-dir scipy pyyaml open3d numpy

   WORKDIR /workspace
   CMD ["/bin/bash"]
   ```

4. **Create the GitHub Actions workflow** — write `.github/workflows/docker-<name>.yml`. Always identical structure (matches existing FAST_LIO / LIO_SAM / genz_icp / mad_icp workflows):

   ```yaml
   name: Docker – <name>

   on:
     push:
       branches: [main, master]
       tags: ["v*.*.*"]
       paths:
         - "docker/<name>/**"
         - ".github/workflows/docker-<name>.yml"
     pull_request:
       branches: [main, master]
       paths:
         - "docker/<name>/**"
         - ".github/workflows/docker-<name>.yml"
     workflow_dispatch:

   jobs:
     build-and-push:
       runs-on: ubuntu-latest
       permissions:
         contents: read
         packages: write
       steps:
         - name: Checkout (with submodules)
           uses: actions/checkout@v4
           with:
             submodules: recursive

         - name: Free disk space
           run: |
             sudo rm -rf /usr/share/dotnet /usr/local/lib/android /opt/ghc /opt/hostedtoolcache
             sudo rm -rf /usr/share/swift /usr/local/graalvm /usr/local/.ghcup
             sudo rm -rf /usr/local/share/powershell /usr/local/share/chromium
             sudo docker image prune -af
             df -h

         - name: Set up Docker Buildx
           uses: docker/setup-buildx-action@v3

         - name: Log in to GHCR
           if: github.event_name != 'pull_request'
           uses: docker/login-action@v3
           with:
             registry: ghcr.io
             username: ${{ github.actor }}
             password: ${{ secrets.GITHUB_TOKEN }}

         - name: Build and push runtime
           uses: docker/build-push-action@v5
           with:
             context: .
             file: docker/<name>/Dockerfile
             push: ${{ github.event_name != 'pull_request' }}
             tags: |
               ghcr.io/wangxinjian1108/<image-name>:latest
               ghcr.io/wangxinjian1108/<image-name>:${{ github.sha }}
             cache-from: type=gha
             cache-to: type=gha,mode=max
   ```

   Image-name conventions:
   - C++ ROS submodules with underscores in name (FAST_LIO, LIO_SAM): map to dash form (`fast-lio`, `lio-sam`) for the GHCR image
   - Python pip-installable: same name as the PyPI package (`genz-icp`, `mad-icp`)

5. **Report** — after writing both files, print a one-line summary: which template (A/B), what apt + source-build deps, the GHCR image tag.

## Hard-won lessons (from existing SlamHub Dockerfiles)

- **Submodule URL strategy**: prefer `wangxinjian1108/<name>` fork as the gitlink URL when patching might be needed; otherwise upstream is fine. Set the gitlink with `git update-index --add --cacheinfo 160000,<sha>,thirdparty/<name>` instead of full local clone (network here is intermittent).
- **Don't `git clone` inside the Dockerfile**. Always COPY. CI runner does the actual `git clone` via `actions/checkout@v4 with submodules: recursive`.
- **GTSAM compile is slow** (~6 min). Build it once in a base layer if multiple SLAM images need it.
- **C++17 sed patch** is needed for any project that pins C++11/14 against PCL 1.10.
- **For pure-Python**: the `thirdparty/<name>` `COPY` is reference-only; we never `pip install /opt/<name>` from the source. We install from PyPI and the source dir is there for audit.
