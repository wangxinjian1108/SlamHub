FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    ROS_DISTRO=noetic

# Standard debug-friendly toolchain (matches FAST_LIO image)
RUN apt-get update && apt-get install -y --no-install-recommends \
    git zsh vim git-lfs wget unzip bzip2 ca-certificates \
    openssh-server clang-format htop iotop rsync ffmpeg curl \
    cmake make less time sqlite3 tree gdb g++ ninja-build \
    build-essential tmux locales lsb-release nano nethogs \
    net-tools valgrind xz-utils sudo pciutils \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8 zh_CN.UTF-8 && update-locale LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8 LANGUAGE=en_US:en LC_ALL=en_US.UTF-8

# ROS deps for LIO-SAM (and PCL/Eigen build chain)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-pcl-ros ros-${ROS_DISTRO}-tf2-eigen \
    ros-${ROS_DISTRO}-rviz ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport ros-${ROS_DISTRO}-eigen-conversions \
    ros-${ROS_DISTRO}-rosbag ros-${ROS_DISTRO}-navigation \
    ros-${ROS_DISTRO}-robot-localization \
    python3-catkin-tools python3-pip python3-rosbag \
    libpcl-dev libeigen3-dev libgflags-dev libgoogle-glog-dev \
    libboost-all-dev libgtsam-dev libgtsam-unstable-dev \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /var/run/sshd /root/.ssh && ssh-keygen -A
RUN pip3 install --no-cache-dir scipy

# Build LIO-SAM
RUN mkdir -p /catkin_ws/src && \
    git clone --depth 1 https://github.com/TixiaoShan/LIO-SAM.git /catkin_ws/src/LIO-SAM

# Patch: latest C++17 toolchain rejects pcl::uint16_t-style template-template
# argument deductions; force LIO-SAM to use C++17 (default Noetic CMake is 14).
RUN sed -i 's|set(CMAKE_CXX_FLAGS "-std=c++14|set(CMAKE_CXX_FLAGS "-std=c++17|g' \
    /catkin_ws/src/LIO-SAM/CMakeLists.txt

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /catkin_ws && \
    catkin_make -j\$(nproc) -DCMAKE_BUILD_TYPE=Release" && \
    test -f /catkin_ws/devel/lib/lio_sam/lio_sam_imageProjection && \
    echo "LIO-SAM built successfully"

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /etc/bash.bashrc

WORKDIR /catkin_ws
CMD ["/bin/bash"]
