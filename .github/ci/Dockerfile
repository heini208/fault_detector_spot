FROM ghcr.io/rai-opensource/spot_ros2_jammy_humble:main

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG ROS_DISTRO=humble
ARG SPOT_ROS2_REF=dbbac7a0afac1766ad43151f8a4b55651066bff1

ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive
ENV QT_QPA_PLATFORM=offscreen

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-pyqt5 \
    python3-pytest \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /tmp/spot_ws/src \
    && git clone --recursive https://github.com/RAI-Opensource/spot_ros2.git /tmp/spot_ws/src/spot_ros2 \
    && cd /tmp/spot_ws/src/spot_ros2 \
    && git checkout "${SPOT_ROS2_REF}" \
    && git submodule update --init --recursive

RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && cd /tmp/spot_ws \
    && colcon build \
        --merge-install \
        --install-base /opt/fault_detector_underlay \
        --packages-up-to \
            spot_driver \
            spot_description \
            spot_msgs \
            spot_examples \
            spot_ros2_control \
            spot_common \
            spot_controllers

COPY package.xml /tmp/fault_detector_spot/package.xml

RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && source /opt/fault_detector_underlay/setup.bash \
    && apt-get update \
    && rosdep update \
    && rosdep install \
        --from-paths /tmp/fault_detector_spot \
        --ignore-src \
        --rosdistro "${ROS_DISTRO}" \
        --skip-keys "spot_ros2 synchros2 bosdyn_msgs spot_msgs spot_wrapper spot_common fault_detector_msgs" \
        -r \
        -y \
    && rm -rf /var/lib/apt/lists/* \
    && rm -rf /tmp/spot_ws /tmp/fault_detector_spot
