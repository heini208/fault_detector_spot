#!/usr/bin/env bash
# Provision a disposable Ubuntu 22.04 / ROS 2 Humble dependency workspace.
# Usage: bash ci/provision_humble.sh /absolute/path/to/new-dependency-workspace
# This installs system packages. Run it only in a disposable CI runner/container.
# The application and fault_detector_msgs are built together by the CI job.
set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 NEW_DEPENDENCY_WORKSPACE" >&2
    exit 2
fi
# shellcheck disable=SC1091
source /etc/os-release
if [[ ${ID} != ubuntu || ${VERSION_ID} != 22.04 || $(uname -m) != x86_64 ]]; then
    echo "This CI provisioning supports Ubuntu 22.04 amd64 only." >&2
    exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "Install the ROS 2 Humble base environment before provisioning." >&2
    exit 2
fi

repository_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
dependency_workspace=$(realpath -m -- "$1")
if [[ -e "$dependency_workspace/src" || -e "$dependency_workspace/install" ]]; then
    echo "Use a new dependency workspace; existing sources/install are preserved." >&2
    exit 2
fi
mkdir -p "$dependency_workspace/src" "$dependency_workspace/downloads" \
    "$dependency_workspace/evidence"

as_root() {
    if [[ ${EUID} -eq 0 ]]; then
        "$@"
    else
        sudo -H -- "$@"
    fi
}
source_ros() {
    # ROS setup scripts do not promise compatibility with nounset.
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    set -u
}
export DEBIAN_FRONTEND=noninteractive
export PYTHONNOUSERSITE=1

as_root apt-get update
as_root apt-get install -y --no-install-recommends \
    build-essential cmake curl git ca-certificates \
    python3-colcon-common-extensions python3-pip python3-rosdep \
    python3-vcstool python3-yaml libspdlog-dev \
    ros-humble-proto2ros
source_ros

vcs import "$dependency_workspace/src" < "$repository_root/ci/humble.repos"
cp "$repository_root/ci/humble.repos" "$dependency_workspace/evidence/provisioning.repos"
cp "$repository_root/ci/patches/spot-wrapper-posture.patch" \
    "$dependency_workspace/evidence/spot-wrapper-posture.patch"
sha256sum "$repository_root/ci/humble.repos" \
    "$repository_root/ci/provision_humble.sh" \
    "$repository_root/ci/patches/spot-wrapper-posture.patch" \
    "$repository_root/ci/requirements-humble.txt" \
    "$repository_root/ci/requirements-pure.txt" \
    "$repository_root/ci/requirements-test.txt" \
    "$repository_root/requirements-humble-sdk.txt" \
    > "$dependency_workspace/evidence/provisioning-inputs.sha256"
/usr/bin/python3 - "$repository_root/ci/humble.repos" "$dependency_workspace/src" \
    <<'PY' | tee "$dependency_workspace/evidence/source-revisions.txt"
from pathlib import Path
import subprocess
import sys
import yaml

manifest = yaml.safe_load(Path(sys.argv[1]).read_text())
for name, specification in manifest["repositories"].items():
    path = Path(sys.argv[2]) / name
    actual = subprocess.check_output(
        ["git", "-C", str(path), "rev-parse", "HEAD"], text=True
    ).strip()
    if actual != specification["version"]:
        raise SystemExit(f"{name}: expected {specification['version']}, got {actual}")
    print(f"{name}: {actual}")
PY

# Preserve the existing driver's successful generic sit/stand feedback behavior.
# This exact patch is required by the real SpotWrapper contract tests. Its base
# revision is pinned in humble.repos; no local development checkout is consulted.
git -C "$dependency_workspace/src/spot_wrapper" apply --check \
    "$repository_root/ci/patches/spot-wrapper-posture.patch"
git -C "$dependency_workspace/src/spot_wrapper" apply \
    "$repository_root/ci/patches/spot-wrapper-posture.patch"

# Download the official release debs directly. The upstream .run installer also
# copies rosdep rules; direct apt installation avoids empty vendor rosdep rules.
# Digests come from the GitHub release API for rai-opensource/bosdyn_msgs 5.0.1.
cat > "$dependency_workspace/downloads/bosdyn.sha256" <<'CHECKSUMS'
da1cc37f23a071b340dacc5009adecdcc7b137730ac44e76dbb0d2385ae45744  ros-humble-bosdyn-api-msgs_5.0.1-0jammy_amd64.deb
81b9ccff631e5d0634a9fe5940b55ad25f8de065c7c63bb3494c0c35c2f8795d  ros-humble-bosdyn-auto-return-api-msgs_5.0.1-0jammy_amd64.deb
be1e1150282a4ac86c03241e625813b51f0307e3ffe37cb20b81974bae92b8cb  ros-humble-bosdyn-autowalk-api-msgs_5.0.1-0jammy_amd64.deb
55624845ce73f33f364daaa44d0762e467ccdefe7451bda52d100ec866aca141  ros-humble-bosdyn-cmake-module_5.0.1-0jammy_amd64.deb
937770c82aa13bb7834e4ad5275f24b56bbc51977c649f9aa96d27d3250c3bce  ros-humble-bosdyn-graph-nav-api-msgs_5.0.1-0jammy_amd64.deb
bc3885f1b17a4c17c1702850bec35d390b12c8dfb67050141a47de2f0fd0155c  ros-humble-bosdyn-keepalive-api-msgs_5.0.1-0jammy_amd64.deb
18876ad6a4a238bdd14eb0212aab91bfba02f3e1f6b95b00aba6f78adb843d2d  ros-humble-bosdyn-log-status-api-msgs_5.0.1-0jammy_amd64.deb
5c0bf2850e8fef0c58f40e6ea50fbe0d6be2b2f263981487a34062f39813c3ad  ros-humble-bosdyn-metrics-logging-api-msgs_5.0.1-0jammy_amd64.deb
f86f53fd31240b2ec4bde2786af4c196188b4a4ad9413b7ec0b07cbab77eb962  ros-humble-bosdyn-mission-api-msgs_5.0.1-0jammy_amd64.deb
7a3c13754af2c425685467fd69f4dc31973043dca04087aa7e03bfa98c8fef1a  ros-humble-bosdyn-msgs_5.0.1-0jammy_amd64.deb
46f956660ef91b288a5cbd71e50453de238f5129cb0329a86bfecbd0150eeed1  ros-humble-bosdyn-spot-api-msgs_5.0.1-0jammy_amd64.deb
3f6c2314b746f7d741a74b4e2f5dbc5d0a727d4c3db0e6be88c44660ccbf2422  ros-humble-bosdyn-spot-cam-api-msgs_5.0.1-0jammy_amd64.deb
CHECKSUMS
cp "$dependency_workspace/downloads/bosdyn.sha256" \
    "$dependency_workspace/evidence/bosdyn.sha256"
while read -r digest filename; do
    curl --fail --location --retry 3 \
        "https://github.com/rai-opensource/bosdyn_msgs/releases/download/5.0.1/$filename" \
        --output "$dependency_workspace/downloads/$filename"
done < "$dependency_workspace/downloads/bosdyn.sha256"
(
    cd "$dependency_workspace/downloads"
    sha256sum --check bosdyn.sha256
)
cpp_deb=spot-cpp-sdk_5.0.1_amd64.deb
curl --fail --location --retry 3 \
    "https://github.com/rai-opensource/spot-cpp-sdk/releases/download/v5.0.1/$cpp_deb" \
    --output "$dependency_workspace/downloads/$cpp_deb"
printf '%s  %s\n' \
    10053b418655c9e594615c39b5a818910147d0d1df3250c2c1932470978e4121 \
    "$dependency_workspace/downloads/$cpp_deb" | sha256sum --check
as_root apt-get install -y "$dependency_workspace/downloads/"*.deb

# The CI lock includes production SDK requirements and its runtime closure.
# spot_wrapper additionally imports bosdyn.mission.
as_root /usr/bin/python3 -m pip install --disable-pip-version-check \
    -r "$repository_root/ci/requirements-humble.txt"
as_root /usr/bin/python3 -m pip install --disable-pip-version-check \
    --no-deps --no-build-isolation "$dependency_workspace/src/spot_wrapper"
source_ros

# Verify real vendor installations before rosdep --ignore-src consults the
# ament index. Missing packages cannot be hidden by skip keys or empty rules.
/usr/bin/python3 - <<'PY'
from importlib.metadata import version
from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory

for distribution in (
    "bosdyn-api", "bosdyn-client", "bosdyn-core", "bosdyn-mission",
    "bosdyn-choreography-client", "bosdyn-choreography-protos",
):
    assert version(distribution) == "5.0.1", distribution
for package in (
    "bosdyn_api_msgs", "bosdyn_spot_api_msgs", "bosdyn_spot_cam_api_msgs",
    "bosdyn_msgs",
):
    xml = Path(get_package_share_directory(package)) / "package.xml"
    assert ET.parse(xml).findtext("version") == "5.0.1", package
from bosdyn_spot_api_msgs.conversions import convert
from spot_wrapper.wrapper import SpotWrapper
assert callable(convert) and callable(SpotWrapper.robot_command)
PY

dependency_paths=(
    "$dependency_workspace/src/spot_ros2/spot_msgs"
    "$dependency_workspace/src/synchros2/synchros2"
    "$dependency_workspace/src/spot_description/spot_description"
    "$dependency_workspace/src/spot_moveit_config"
    "$dependency_workspace/src/micro_ros_agent/micro_ros_agent"
    "$dependency_workspace/src/micro_ros_msgs"
)
rosdep_paths=(
    "${dependency_paths[@]}"
    "$dependency_workspace/src/fault_detector_msgs"
    "$repository_root"
)
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    as_root rosdep init
fi
rosdep update --rosdistro humble
rosdep keys --from-paths "${rosdep_paths[@]}" --ignore-src --rosdistro humble \
    > "$dependency_workspace/evidence/rosdep-keys.txt"
rosdep install --from-paths "${rosdep_paths[@]}" --ignore-src \
    --rosdistro humble --yes 2>&1 \
    | tee "$dependency_workspace/evidence/rosdep-install.log"
rosdep check --from-paths "${rosdep_paths[@]}" --ignore-src --rosdistro humble \
    2>&1 | tee "$dependency_workspace/evidence/rosdep-check.log"

# Do not discover/build the unrelated driver, tutorials, or its dependencies.
# The pinned micro-ROS Agent superbuild selects Micro-XRCE-DDS-Agent v2.4.2;
# use Jammy's spdlog instead of downloading a moving logger dependency.
(
    cd "$dependency_workspace"
    colcon build --base-paths "${dependency_paths[@]}" \
        --executor sequential --event-handlers console_direct+ \
        --cmake-args -DBUILD_TESTING=OFF -DUAGENT_USE_SYSTEM_LOGGER=ON \
        2>&1 | tee "$dependency_workspace/evidence/dependency-build.log"
)
xrce_revision=$(git -C "$dependency_workspace/build/micro_ros_agent/agent/src/xrceagent" \
    rev-parse HEAD)
if [[ $xrce_revision != 57d086216d01ec43121845d385894a25987f8a2c ]]; then
    echo "Unexpected Micro-XRCE-DDS-Agent v2.4.2 revision: $xrce_revision" >&2
    exit 1
fi
vcs export --exact "$dependency_workspace/src" \
    > "$dependency_workspace/evidence/resolved-sources.repos"
vcs export --exact "$dependency_workspace/build" \
    > "$dependency_workspace/evidence/resolved-build-sources.repos"
printf 'Dependency overlay: %s/install/setup.bash\n' "$dependency_workspace"
printf 'Build together next: %s and %s/src/fault_detector_msgs\n' \
    "$repository_root" "$dependency_workspace"
