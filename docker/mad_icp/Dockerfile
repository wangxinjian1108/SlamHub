# MAD-ICP — RA-L 2024 minimal/robust/realtime LiDAR odometry, RVP group, Sapienza.
# Upstream ships a pip wheel; native build needs Eigen + OpenMP + pybind11 (the
# pip install pulls in pybind11 and FetchContents the rest, but pip's sdist
# build needs ninja). Image is a thin Ubuntu 22.04 + pip install mad-icp,
# matching genz_icp / kiss-icp deployment cost.
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1

# Standard debug-friendly toolchain (matches FAST_LIO / LIO_SAM / genz_icp images)
RUN apt-get update && apt-get install -y --no-install-recommends \
    git zsh vim git-lfs wget unzip bzip2 ca-certificates \
    openssh-server clang-format htop iotop rsync ffmpeg curl \
    cmake make less time sqlite3 tree gdb g++ ninja-build \
    build-essential tmux locales lsb-release nano nethogs \
    net-tools valgrind xz-utils sudo pciutils \
    python3 python3-pip python3-dev python3-venv \
    libeigen3-dev libomp-dev \
    libgl1 libgomp1 \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8 zh_CN.UTF-8 && update-locale LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8 LANGUAGE=en_US:en LC_ALL=en_US.UTF-8

RUN mkdir -p /var/run/sshd /root/.ssh && ssh-keygen -A

# Bring in upstream mad-icp source from the SlamHub git submodule. We don't
# build from this checkout — pip grabs the wheel — but the source is here for
# reproducibility, to match the wheel SHA, and to allow patches if needed.
COPY thirdparty/mad-icp /opt/mad-icp

# Pin to a specific wheel; bump in lockstep with the submodule SHA.
# click<8.2 is required: typer 0.12.x in the dependency closure trips on
# click >=8.2's stricter "Secondary flag is not valid for non-boolean flag"
# check, and mad_icp's `--realtime/--no-realtime` flag pattern is exactly
# what gets rejected. Pinning click<8.2 is the lightest-weight workaround.
RUN pip3 install --no-cache-dir ninja && \
    pip3 install --no-cache-dir mad-icp==0.0.10 "click<8.2"

# Sanity check at build time so a broken wheel surfaces in CI, not at runtime.
RUN mad_icp --help >/dev/null && python3 -c "import mad_icp; print('mad_icp OK')"

# Useful runtime extras for SlamHub's downstream pipeline.
RUN pip3 install --no-cache-dir scipy pyyaml open3d numpy

WORKDIR /workspace
CMD ["/bin/bash"]
