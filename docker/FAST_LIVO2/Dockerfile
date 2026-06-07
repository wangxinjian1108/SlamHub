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
#   1. C++11 made std::complex<T>::real() and imag() return rvalues — `x.real() = 0.;`
#      is a build error. Replace with the new setter form `x.real(0.)`.
#   2. Same fix for both so2.cpp and the so2 default constructor.
COPY thirdparty/Sophus /tmp/Sophus
RUN cd /tmp/Sophus && \
    sed -i -E 's|(unit_complex_\.real\(\)) = ([^;]+);|\1.real(\2);|g; s|(unit_complex_\.imag\(\)) = ([^;]+);|\1.imag(\2);|g' sophus/so2.cpp && \
    sed -i -E 's|(unit_complex_nonconst\(\)\.real\(\)) = ([^;]+);|unit_complex_nonconst().real(\2);|g; s|(unit_complex_nonconst\(\)\.imag\(\)) = ([^;]+);|unit_complex_nonconst().imag(\2);|g' sophus/so2.cpp && \
    grep -n "real\|imag" sophus/so2.cpp | head -10 && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && make install && \
    rm -rf /tmp/Sophus && \
    ldconfig

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
