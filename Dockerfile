# Docker image for GS-LIVM with ROS Noetic and CUDA
FROM nvidia/cuda:11.8.0-devel-ubuntu20.04

# Configure environment and install dependencies
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=noetic

# Install ROS, build tooling and CUDA compiler
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        curl gnupg2 lsb-release build-essential \
        cmake python3-catkin-tools python3-rosdep && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends ros-noetic-desktop-full && \
    rm -rf /var/lib/apt/lists/* && \
    rosdep init && \
    rosdep update

# Source ROS environment for future sessions
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

# Copy repository into workspace
WORKDIR /workspace/GS-LIVM
COPY . /workspace/GS-LIVM

# Setup catkin workspace and verify toolchain
RUN mkdir -p /workspace/catkin_ws/src && \
    ln -s /workspace/GS-LIVM /workspace/catkin_ws/src/GS-LIVM && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    cd /workspace/catkin_ws && \
    catkin init && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin build && \
    nvcc --version

CMD ["/bin/bash"]
