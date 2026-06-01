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
    libboost-all-dev libtbb-dev libmetis-dev \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /var/run/sshd /root/.ssh && ssh-keygen -A
RUN pip3 install --no-cache-dir scipy

# Build GTSAM 4.0.3 from source (LIO-SAM's tested version; not in Ubuntu 20.04 apt).
RUN git clone --branch 4.0.3 --depth 1 https://github.com/borglab/gtsam.git /tmp/gtsam && \
    cd /tmp/gtsam && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DGTSAM_BUILD_TESTS=OFF \
          -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON \
          -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_POSE3_EXPMAP=ON \
          -DGTSAM_ROT3_EXPMAP=ON .. && \
    make -j$(nproc) && make install && \
    rm -rf /tmp/gtsam && \
    ldconfig

# Bring in LIO-SAM source from the SlamHub git submodule (no in-image clone).
# Repo root is the docker build context, so thirdparty/LIO-SAM/* is available.
RUN mkdir -p /catkin_ws/src
COPY thirdparty/LIO-SAM /catkin_ws/src/LIO-SAM

# Patch 1: LIO-SAM CMakeLists.txt pins -std=c++11, but PCL 1.10 on Ubuntu 20.04
# requires C++14+. Bump to C++17 (works with both PCL and GTSAM 4.0.3).
# Use a permissive sed that matches c++11 / c++14 alike.
RUN sed -i -E 's|-std=c\+\+1[14]|-std=c++17|g' \
    /catkin_ws/src/LIO-SAM/CMakeLists.txt && \
    grep -E 'std=c\+\+' /catkin_ws/src/LIO-SAM/CMakeLists.txt

# Patch 2: replace legacy <opencv/cv.h> (OpenCV 2 path) with <opencv2/opencv.hpp>
# Ubuntu 20.04 ships OpenCV 4 which no longer provides the legacy compat header.
RUN sed -i 's|#include <opencv/cv.h>|#include <opencv2/opencv.hpp>|g' \
    /catkin_ws/src/LIO-SAM/include/utility.h

# Patch 3: FLANN 1.9 on Ubuntu 20.04 fails to compile against C++17 + PCL 1.10
# because /usr/include/flann/util/serialization.h calls m.serialize(ar) on
# std::unordered_map, which has no such member. Boost provides a free
# serialize() overload via <boost/serialization/unordered_map.hpp> — include
# it at the very top of utility.h so all downstream PCL/FLANN/Boost-archive
# code in LIO-SAM picks it up before flann tries the call.
RUN sed -i '1i #include <boost/serialization/unordered_map.hpp>' \
    /catkin_ws/src/LIO-SAM/include/utility.h

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /catkin_ws && \
    catkin_make -j\$(nproc) -DCMAKE_BUILD_TYPE=Release" && \
    test -f /catkin_ws/devel/lib/lio_sam/lio_sam_imageProjection && \
    echo "LIO-SAM built successfully"

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /etc/bash.bashrc

WORKDIR /catkin_ws
CMD ["/bin/bash"]
