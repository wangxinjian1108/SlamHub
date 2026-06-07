FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    ROS_DISTRO=noetic

# Standard debug-friendly toolchain (matches FAST_LIO / LIO_SAM Dockerfiles)
RUN apt-get update && apt-get install -y --no-install-recommends \
    git zsh vim git-lfs wget unzip bzip2 ca-certificates \
    openssh-server clang-format htop iotop rsync ffmpeg curl \
    cmake make less time sqlite3 tree gdb g++ ninja-build \
    build-essential tmux locales lsb-release nano nethogs \
    net-tools valgrind xz-utils sudo pciutils \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8 zh_CN.UTF-8 && update-locale LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8 LANGUAGE=en_US:en LC_ALL=en_US.UTF-8

# ROS + SLAM + visual deps. FAST-LIVO2 needs:
#   PCL>=1.8 (apt has 1.10), Eigen>=3.3.4 (apt has 3.3.7), OpenCV>=4.2 (apt has 4.2),
#   image_transport + cv_bridge + pcl_ros + tf (catkin deps via package.xml)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-pcl-ros ros-${ROS_DISTRO}-tf2-eigen \
    ros-${ROS_DISTRO}-rviz ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport ros-${ROS_DISTRO}-eigen-conversions \
    ros-${ROS_DISTRO}-rosbag \
    python3-catkin-tools python3-pip python3-rosbag \
    libpcl-dev libeigen3-dev libgflags-dev libgoogle-glog-dev \
    libboost-all-dev libtbb-dev libgtest-dev \
    libopencv-dev libopencv-contrib-dev \
    libfmt-dev \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /var/run/sshd /root/.ssh && ssh-keygen -A
RUN pip3 install --no-cache-dir scipy numpy

# Build Sophus from source. FAST-LIVO2 README pins commit a621ff (== a621ff2),
# the non-templated/double-only version. Newer Sophus templated API breaks the
# code. We pin via the SlamHub gitlink so the COPY below carries the exact commit.
#
# Patches needed against modern compilers/Eigen:
#   C++11 made std::complex<T>::real() and imag() return rvalues, so
#   `x.real() = 0.;` is no longer a valid lvalue. Replace with the new
#   setter form `x.real(0.);`. The pattern `.real() = X;` → `.real(X);`
#   (and same for imag()) covers all hit sites in so2.cpp.
COPY thirdparty/Sophus /tmp/Sophus
RUN cd /tmp/Sophus && \
    sed -i -E 's|\.real\(\)[[:space:]]*=[[:space:]]*([^;]+);|.real(\1);|g; s|\.imag\(\)[[:space:]]*=[[:space:]]*([^;]+);|.imag(\1);|g' sophus/so2.cpp && \
    grep -n -E "\.(real|imag)\(" sophus/so2.cpp && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && make install && \
    rm -rf /tmp/Sophus && \
    ldconfig

# Old Sophus (a621ff2) doesn't install a config file. vikit_common does
# `FIND_PACKAGE(Sophus REQUIRED)` which prefers config-mode and only falls
# back to module-mode under specific conditions. Install BOTH a module-mode
# FindSophus.cmake AND a config-mode SophusConfig.cmake so any consumer
# strategy works. Use printf instead of heredoc — classic Dockerfile syntax
# doesn't support heredoc inside a chained `RUN ... && \` block.
RUN mkdir -p /usr/local/share/cmake/Modules /usr/local/lib/cmake/Sophus && \
    printf '%s\n' \
        '# Module-mode FindSophus.cmake for source-installed Sophus a621ff2.' \
        'find_path(Sophus_INCLUDE_DIRS sophus/so3.h HINTS /usr/local/include /usr/include)' \
        'find_library(Sophus_LIBRARIES NAMES Sophus HINTS /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu)' \
        'include(FindPackageHandleStandardArgs)' \
        'find_package_handle_standard_args(Sophus DEFAULT_MSG Sophus_INCLUDE_DIRS Sophus_LIBRARIES)' \
        > /usr/local/share/cmake/Modules/FindSophus.cmake && \
    printf '%s\n' \
        '# Config-mode SophusConfig.cmake for source-installed Sophus a621ff2.' \
        'set(Sophus_INCLUDE_DIRS "/usr/local/include")' \
        'set(Sophus_LIBRARIES "/usr/local/lib/libSophus.so")' \
        'set(Sophus_FOUND TRUE)' \
        'mark_as_advanced(Sophus_INCLUDE_DIRS Sophus_LIBRARIES)' \
        > /usr/local/lib/cmake/Sophus/SophusConfig.cmake && \
    cat /usr/local/share/cmake/Modules/FindSophus.cmake && \
    echo "---" && \
    cat /usr/local/lib/cmake/Sophus/SophusConfig.cmake
ENV CMAKE_MODULE_PATH=/usr/local/share/cmake/Modules

# Build catkin workspace with FAST-LIVO2 + rpg_vikit + livox_ros_driver, in this order.
# Reasons we COPY instead of git clone:
#   - reproducibility: pinned via SlamHub gitlinks
#   - network: no internet inside build context
RUN mkdir -p /catkin_ws/src
COPY thirdparty/livox_ros_driver /catkin_ws/src/livox_ros_driver
COPY thirdparty/rpg_vikit /catkin_ws/src/rpg_vikit
COPY thirdparty/FAST-LIVO2 /catkin_ws/src/FAST-LIVO2

# Two-pass build:
#   1) livox_ros_driver alone — needed for message generation (Pose6D etc.)
#   2) full build — vikit + FAST-LIVO2 link against livox_ros_driver headers
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /catkin_ws && \
    catkin_make -j\$(nproc) -DCATKIN_WHITELIST_PACKAGES='livox_ros_driver' && \
    catkin_make -j\$(nproc) -DCATKIN_WHITELIST_PACKAGES='' -DCMAKE_BUILD_TYPE=Release" && \
    test -f /catkin_ws/devel/lib/fast_livo/fastlivo_mapping && \
    echo "FAST-LIVO2 built successfully"

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /etc/bash.bashrc

WORKDIR /catkin_ws
CMD ["/bin/bash"]
