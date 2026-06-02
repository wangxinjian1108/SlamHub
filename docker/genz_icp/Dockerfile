# GenZ-ICP — RA-L 2025 degeneracy-robust LiDAR odometry, KISS-ICP-style pipeline.
# Upstream provides a pip package as of v0.3.x, so the image is a thin
# Python-on-Ubuntu base with `pip install genz-icp[all]` plus the same
# debug-friendly toolchain shared with FAST_LIO / LIO_SAM images.
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1

# Standard debug-friendly toolchain (matches FAST_LIO / LIO_SAM images)
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

# Bring in upstream genz-icp source from the SlamHub git submodule. We don't
# build anything from this checkout — pip install grabs the released wheel.
# The submodule is here for reproducibility and to pin the source rev that
# matches the wheel we install (so tests / patches can target this tree).
COPY thirdparty/genz-icp /opt/genz-icp

# Install GenZ-ICP from PyPI with full visualizer extras (open3d / polyscope).
# Pinning here matches whatever wheel is current; bump in lockstep with the
# submodule SHA when refreshing.
RUN pip3 install --no-cache-dir "genz-icp[all]==0.3.2"

# Sanity check at build time so a broken wheel surfaces in CI, not at runtime.
RUN genz_icp_pipeline --version && \
    python3 -c "import genz_icp, open3d, numpy; print('genz_icp', genz_icp.__version__)"

# Useful runtime extras for SlamHub's downstream pipeline (reading PCDs,
# trajectory composition, cross-LiDAR ICP target prep). Already pulled in by
# genz-icp[all] but listed explicitly for clarity.
RUN pip3 install --no-cache-dir scipy pyyaml

WORKDIR /workspace
CMD ["/bin/bash"]
