#!/usr/bin/env bash
# Build the reviewed RViz lifetime fix as a persistent, isolated overlay.

check_moveit_versions() {
    /usr/bin/python3 - <<'PY'
from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory

for package in ('moveit_core', 'moveit_ros_planning', 'moveit_ros_visualization'):
    path = Path(get_package_share_directory(package)) / 'package.xml'
    version = ET.parse(path).findtext('version')
    if version != '2.5.10':
        raise SystemExit(f'RViz hotfix requires {package} 2.5.10; installed version is {version}')
PY
}

apply_moveit_hotfix() {
    local source_dir=$1 patch_file=$2
    if patch --batch --forward --fuzz=0 --dry-run -d "$source_dir" -p1 \
            < "$patch_file" >/dev/null 2>&1; then
        patch --batch --forward --fuzz=0 -d "$source_dir" -p1 < "$patch_file"
    elif patch --batch --reverse --fuzz=0 --dry-run -d "$source_dir" -p1 \
            < "$patch_file" >/dev/null 2>&1; then
        printf 'RViz lifetime patch is already applied.\n'
    else
        printf 'RViz lifetime patch conflicts or is partially applied; inspect %s. Sources preserved.\n' \
            "$source_dir" >&2
        return 1
    fi
}

main() {
    set -euo pipefail
    [[ $# -eq 1 ]] || { echo 'Usage: install_moveit_hotfix.sh WORKSPACE' >&2; return 2; }
    [[ ${ROS_DISTRO:-} == humble ]] || { echo 'Source ROS 2 Humble first.' >&2; return 2; }
    check_moveit_versions
    local workspace repository_root hotfix_root archive source_dir staging digest
    workspace=$(realpath -- "$1")
    repository_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
    hotfix_root="$workspace/hotfixes"
    archive="$hotfix_root/moveit2-2.5.10.tar.gz"
    source_dir="$hotfix_root/moveit2-2.5.10"
    digest=f8606f3ead034f989e9adccc98943eeddb1d3eb09d18f7a4092534456b5f5ae0
    mkdir -p "$hotfix_root"
    if [[ ! -e "$archive" ]]; then
        curl --fail --location --retry 3 \
            https://codeload.github.com/moveit/moveit2/tar.gz/refs/tags/2.5.10 \
            --output "$archive.part"
        printf '%s  %s\n' "$digest" "$archive.part" | sha256sum --check
        mv -- "$archive.part" "$archive"
    fi
    printf '%s  %s\n' "$digest" "$archive" | sha256sum --check
    if [[ ! -e "$source_dir" ]]; then
        staging=$(mktemp -d "$hotfix_root/.extract-XXXXXX")
        tar -xzf "$archive" -C "$staging"
        mv -- "$staging/moveit2-2.5.10" "$source_dir"
        rmdir -- "$staging"
    fi
    apply_moveit_hotfix "$source_dir" \
        "$repository_root/patches/moveit-2.5.10/rviz-scene-monitor-lifetime.patch"
    rosdep install --from-paths "$source_dir/moveit_ros/visualization" \
        --ignore-src --rosdistro humble --yes
    rosdep check --from-paths "$source_dir/moveit_ros/visualization" \
        --ignore-src --rosdistro humble
    check_moveit_versions
    /usr/bin/python3 "$repository_root/patches/moveit-2.5.10/check_lifetime.py" "$hotfix_root"
    CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL:-4} \
        colcon --log-base "$hotfix_root/log" build \
        --base-paths "$source_dir/moveit_ros/visualization" \
        --packages-select moveit_ros_visualization --allow-overriding moveit_ros_visualization \
        --build-base "$hotfix_root/build" --install-base "$hotfix_root/install" \
        --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF
    printf 'RViz hotfix overlay: %s/install/local_setup.bash\n' "$hotfix_root"
}

if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
    main "$@"
fi
