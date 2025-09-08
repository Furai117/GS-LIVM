FROM nvidia/cuda:11.7.1-devel-ubuntu20.04

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install ROS Noetic and development tools
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg2 lsb-release build-essential cmake git python3-catkin-tools \
        python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool \
    && curl -sSL "https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc" | apt-key add - \
    && echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends ros-noetic-desktop-full \
    && rosdep init \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=noetic

SHELL ["/bin/bash", "-c"]

# Source ROS environment by default
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

# Verify CUDA availability
RUN nvcc --version

# Set up catkin workspace and build repository
WORKDIR /root/catkin_ws
RUN mkdir -p src
COPY . /root/catkin_ws/src/GS-LIVM
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    catkin init && \
    catkin build

CMD ["/bin/bash"]
