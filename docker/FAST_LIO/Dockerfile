FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    ROS_DISTRO=noetic

WORKDIR /fast_lio

RUN apt-get update && apt-get install -y --no-install-recommends \
    git zsh vim git-lfs wget unzip bzip2 ca-certificates \
    openssh-server clang-format htop iotop rsync ffmpeg curl \
    cmake make less time sqlite3 tree gdb g++ ninja-build \
    build-essential tmux locales lsb-release nano nethogs \
    net-tools valgrind xz-utils sudo pciutils \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8 zh_CN.UTF-8 && update-locale LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8 LANGUAGE=en_US:en LC_ALL=en_US.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-pcl-ros ros-${ROS_DISTRO}-tf2-eigen \
    ros-${ROS_DISTRO}-rviz ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport ros-${ROS_DISTRO}-eigen-conversions \
    python3-catkin-tools python3-pip python3-rosbag \
    ros-${ROS_DISTRO}-rosbag \
    libpcl-dev libeigen3-dev libgflags-dev libgoogle-glog-dev \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /var/run/sshd /root/.ssh && ssh-keygen -A
RUN pip3 install --no-cache-dir scipy

RUN mkdir -p /catkin_ws/src && \
    git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver.git /catkin_ws/src/livox_ros_driver && \
    git clone --depth 1 https://github.com/hku-mars/FAST_LIO.git /catkin_ws/src/FAST_LIO && \
    cd /catkin_ws/src/FAST_LIO && git submodule update --init --recursive

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /catkin_ws && \
    catkin_make -j\$(nproc) -DCATKIN_WHITELIST_PACKAGES='livox_ros_driver' && \
    catkin_make -j\$(nproc) -DCATKIN_WHITELIST_PACKAGES='' -DCMAKE_BUILD_TYPE=Release" && \
    test -f /catkin_ws/devel/lib/fast_lio/fastlio_mapping && \
    echo "FAST-LIO2 built successfully"

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /etc/bash.bashrc

WORKDIR /catkin_ws
CMD ["/bin/bash"]
