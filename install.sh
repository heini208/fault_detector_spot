#!/usr/bin/env bash
# Download into a new workspace, then run: bash install.sh
# Installs dependencies and builds; it never starts ROS nodes or connects to Spot.

as_root() {
    if [[ ${EUID} -eq 0 ]]; then "$@"; else sudo -H -- "$@"; fi
}

check_platform() {
    # shellcheck disable=SC1091
    source /etc/os-release
    if [[ ${ID} != ubuntu || ${VERSION_ID} != 22.04 || $(uname -m) != x86_64 ]]; then
        echo "This installer supports Ubuntu 22.04 amd64 with ROS 2 Humble." >&2
        return 2
    fi
}

clean_ros_environment() {
    unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_DISTRO ROS_VERSION \
        ROS_PYTHON_VERSION ROS_ROOT ROS_PACKAGE_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH \
        LIBRARY_PATH CPATH PKG_CONFIG_PATH VIRTUAL_ENV CONDA_PREFIX \
        AMENT_CURRENT_PREFIX COLCON_CURRENT_PREFIX
    export PATH=/usr/sbin:/usr/bin:/sbin:/bin PYTHONNOUSERSITE=1 LANG=C.UTF-8 LC_ALL=C.UTF-8
}

humble_available() (
    clean_ros_environment
    [[ -f /opt/ros/humble/setup.bash ]] || return 1
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash >/dev/null 2>&1 || return 1
    [[ ${ROS_DISTRO:-} == humble ]] || return 1
    /usr/bin/python3 -c 'import rclpy; assert rclpy.__file__.startswith("/opt/ros/humble/")' \
        >/dev/null 2>&1
)

install_humble() {
    local workspace=$1 version archive
    echo "Installing ROS 2 Humble and its repository configuration."
    as_root apt-get update
    as_root apt-get install -y --no-install-recommends \
        ca-certificates curl git python3 software-properties-common
    as_root add-apt-repository -y universe
    version=$(curl --fail --silent --show-error --location --retry 3 \
        https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | /usr/bin/python3 -c 'import json, sys; print(json.load(sys.stdin)["tag_name"])')
    [[ $version =~ ^[0-9][[:alnum:]._-]*$ ]] || {
        echo "Invalid ros2-apt-source release version: $version" >&2; return 1;
    }
    mkdir -p "$workspace/downloads"
    archive="$workspace/downloads/ros2-apt-source_${version}.jammy_all.deb"
    curl --fail --location --retry 3 \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/$version/ros2-apt-source_${version}.jammy_all.deb" \
        --output "$archive.part"
    mv -- "$archive.part" "$archive"
    as_root dpkg -i "$archive"
    as_root apt-get update
    # Early Jammy systemd/udev mismatches can otherwise remove desktop packages.
    as_root apt-get install -y --no-remove --only-upgrade systemd udev libsystemd0 libudev1
    # Reinstall the setup scripts and Python entry point if an existing ROS
    # installation failed the source/import check.
    as_root apt-get install -y --reinstall ros-humble-ros-base \
        ros-humble-ros-workspace ros-humble-ros-environment ros-humble-rclpy
}

prepare_application() {
    local workspace=$1 application="$1/src/fault_detector_spot"
    if ! command -v git >/dev/null; then
        as_root apt-get update
        as_root apt-get install -y git ca-certificates
    fi
    if [[ ! -e $application ]]; then
        mkdir -p "$workspace/src"
        git clone https://github.com/heini208/fault_detector_spot.git "$application"
    fi
    if [[ $(git -C "$application" rev-parse --show-toplevel) != "$application" \
        || ! -f "$application/ci/provision_humble.sh" ]]; then
        echo "Expected a fault_detector_spot Git checkout at $application; existing files were preserved." >&2
        return 1
    fi
}

write_manual_steps() {
    local workspace=$1 config="$1/config/spot_ros.yaml"
    mkdir -p "$workspace/config"
    if [[ ! -e $config ]]; then
        (umask 077; cp -- "$workspace/src/spot_ros2/spot_driver/config/spot_ros_example.yaml" "$config")
    fi
    printf '\nBuild complete. Before your first robot session:\n'
    printf '1. Edit %s\n' "$config"
    printf '   Set ros__parameters.username, password, and hostname (Spot IP address).\n'
    printf '   For an EAP Velodyne, set use_velodyne: true. Keep auto_power_on/auto_stand false.\n'
    printf '   Connect this computer to the Spot network and verify the configured IP address.\n'
    printf '   Configure your sensor calibration and frame transforms before using sensor data.\n'
    printf '2. In each fresh terminal, source:\n'
    printf '   source %q\n' "$workspace/setup.bash"
    printf '   This includes ROS Humble, the workspace, and the patched RViz overlay.\n'
    printf '3. Start the driver manually when ready:\n'
    printf '   ros2 launch spot_driver spot_driver.launch.py config_file:=%q\n' "$config"
    printf '4. In another sourced terminal, start the application:\n'
    printf '   ros2 launch fault_detector_spot fault_detector_launch.py\n'
}

main() (
    set -euo pipefail
    [[ $# -eq 0 ]] || { echo "Usage: run bash install.sh inside your workspace" >&2; exit 2; }
    check_platform
    clean_ros_environment
    local workspace
    workspace=$(pwd -P)
    if ! humble_available; then install_humble "$workspace"; fi
    humble_available || { echo "ROS Humble still cannot be sourced/imported; installation stopped." >&2; exit 1; }
    prepare_application "$workspace"
    bash "$workspace/src/fault_detector_spot/ci/provision_humble.sh" "$workspace" --runtime
    write_manual_steps "$workspace"
)

if [[ ${BASH_SOURCE[0]} == "$0" ]]; then main "$@"; fi
