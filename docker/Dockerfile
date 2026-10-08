# Simulation + flight stack for the 3D planner project.
# Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic / PX4 SITL / Micro XRCE-DDS Agent.
FROM osrf/ros:jazzy-desktop

ARG PX4_VERSION=v1.17.0
ARG PX4_MSGS_BRANCH=release/1.17
ARG XRCE_AGENT_VERSION=v2.4.3

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Gazebo Harmonic (the version PX4 1.17 and ros_gz on Jazzy target).
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg lsb-release sudo git wget ca-certificates \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic \
        ros-jazzy-ros-gz \
        python3-colcon-common-extensions \
        python3-pip \
        ros-jazzy-ament-cmake-gtest \
    && rm -rf /var/lib/apt/lists/*

# Micro XRCE-DDS Agent: bridges PX4 uORB topics to ROS 2.
RUN git clone --depth 1 --branch ${XRCE_AGENT_VERSION} \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/xrce \
    && cmake -S /tmp/xrce -B /tmp/xrce/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/xrce/build -j"$(nproc)" \
    && cmake --install /tmp/xrce/build \
    && ldconfig && rm -rf /tmp/xrce

# The rest is owned by the non-root user (UID 1000, matches the host user)
# so PX4 can write its rootfs at runtime and mounted files keep host ownership.
RUN echo "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu \
    && mkdir -p /opt/PX4-Autopilot /opt/px4_ws/src \
    && chown -R ubuntu:ubuntu /opt/PX4-Autopilot /opt/px4_ws
USER ubuntu
WORKDIR /home/ubuntu

# PX4 SITL. ubuntu.sh installs the build toolchain + python deps; skip NuttX (no real hardware).
RUN git clone --depth 1 --branch ${PX4_VERSION} --recurse-submodules --shallow-submodules \
        https://github.com/PX4/PX4-Autopilot.git /opt/PX4-Autopilot \
    && sudo apt-get update \
    && bash /opt/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx \
    && sudo rm -rf /var/lib/apt/lists/*
RUN cd /opt/PX4-Autopilot && make px4_sitl_default

# px4_msgs must match the PX4 version, otherwise message hashes/versions disagree.
RUN git clone --depth 1 --branch ${PX4_MSGS_BRANCH} \
        https://github.com/PX4/px4_msgs.git /opt/px4_ws/src/px4_msgs \
    && source /opt/ros/jazzy/setup.bash \
    && cd /opt/px4_ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

# Mapping. Kept after the PX4 layers so adding packages doesn't rebuild PX4.
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
        ros-jazzy-octomap-server \
    && sudo rm -rf /var/lib/apt/lists/*

# The NVIDIA container toolkit mounts libEGL_nvidia but not its glvnd vendor file, so EGL
# would hand the NVIDIA device to Mesa and Gazebo's headless sensor rendering fails.
RUN echo '{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}}' \
    | sudo tee /usr/share/glvnd/egl_vendor.d/10_nvidia.json >/dev/null

ENV PX4_DIR=/opt/PX4-Autopilot
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
    && echo "source /opt/px4_ws/install/setup.bash" >> ~/.bashrc \
    && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> ~/.bashrc

WORKDIR /ws
CMD ["bash"]
