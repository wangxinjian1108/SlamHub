ARG BASE_IMAGE=ghcr.io/wangxinjian1108/slamhub-base:cuda-ros1
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=noetic

# FAST_LIO dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-tf2-eigen \
    ros-${ROS_DISTRO}-rviz \
    libpcl-dev \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    && rm -rf /var/lib/apt/lists/*

# Create catkin workspace
RUN mkdir -p /catkin_ws/src

# livox_ros_driver (ROS1 version for Livox LiDAR support)
WORKDIR /catkin_ws/src
RUN git clone https://github.com/Livox-SDK/livox_ros_driver.git

# FAST_LIO source
COPY thirdparty/FAST_LIO /catkin_ws/src/FAST_LIO

# Build workspace
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin_make -DCMAKE_BUILD_TYPE=Release"

RUN echo "source /catkin_ws/devel/setup.bash" >> /etc/bash.bashrc

WORKDIR /catkin_ws
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && source /catkin_ws/devel/setup.bash && exec bash"]
