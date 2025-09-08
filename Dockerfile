# Docker image for GS-LIVM with ROS Noetic and CUDA
FROM nvidia/cuda:11.8.0-devel-ubuntu20.04

# Non-interactive frontend
ENV DEBIAN_FRONTEND=noninteractive

# Install required packages and ROS repositories
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release build-essential cmake python3-catkin-tools python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1.list && \
    apt-get update && apt-get install -y ros-noetic-desktop-full && \
    rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=noetic
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

# Copy repository
WORKDIR /opt/GS-LIVM
COPY . /opt/GS-LIVM

# Setup catkin workspace and build
RUN mkdir -p /opt/catkin_ws/src && \
    ln -s /opt/GS-LIVM /opt/catkin_ws/src/GS-LIVM && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    cd /opt/catkin_ws && \
    catkin init && \
    catkin build && \
    nvcc --version

CMD ["/bin/bash"]
