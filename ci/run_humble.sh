#!/usr/bin/env bash
# Build both application packages, verify a copied install, then run offline tests.
set -eo pipefail
source_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
python=${CI_PYTHON:-/usr/bin/python3}
messages_source=${1:?Usage: run_humble.sh MESSAGES_SOURCE NEW_BUILD_WORKSPACE RESULTS}
build_workspace=${2:?Pass a new, empty workspace}
results=${3:?Pass a results directory}
messages_source=$(realpath "$messages_source")
build_workspace=$(realpath -m "$build_workspace")
results=$(realpath -m "$results")
[[ ${ROS_DISTRO:-} == humble ]] || { echo 'Source ROS 2 Humble first' >&2; exit 1; }
[[ ! -e "$build_workspace" ]] || { echo 'Build workspace must be new' >&2; exit 1; }
mkdir -p "$build_workspace" "$results"
export QT_QPA_PLATFORM=offscreen PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1
export ROS_HOME="$results/ros-home"

"$python" "$source_root/ci/record_environment.py" --output "$results" \
    --dependency "$messages_source"
"$python" -m ruff check --select F "$source_root/fault_detector_spot" "$source_root/launch" \
    | tee "$results/ruff.log"
cd "$build_workspace"
"$python" -m colcon --log-base "$results/colcon" build \
    --base-paths "$messages_source" "$source_root" \
    --packages-select fault_detector_msgs fault_detector_spot \
    --executor sequential --event-handlers console_direct+ \
    --cmake-args "-DPython3_EXECUTABLE=$python" "-DPYTHON_EXECUTABLE=$python" \
    |& tee "$results/build.log"
source "$build_workspace/install/local_setup.bash"
cd "$results"
"$python" "$source_root/scripts/check_install.py" \
    --prefix "$build_workspace/install/fault_detector_spot" \
    --messages-prefix "$build_workspace/install/fault_detector_msgs" \
    --source-tree "$source_root" |& tee "$results/imports.log"
cd "$source_root"
# This is the full offline suite. The preview executor test creates actual nodes.
"$python" -m pytest -p no:rerunfailures -q test \
    --ignore=test/test_preview_executor_responsiveness.py \
    -o "cache_dir=$results/pytest-cache" --basetemp="$results/pytest-tmp" \
    --junitxml="$results/results.xml" |& tee "$results/tests.log"
