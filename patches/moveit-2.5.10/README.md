# RViz scene-monitor lifetime fix (MoveIt 2.5.10)

The captured RViz crash has `onSceneMonitorReceivedUpdate()` running while the
GUI thread destroys its `PlanningSceneMonitor` during model reload. The callback
dereferences the display's monitor pointer after `shared_ptr::reset()` clears it.

This patch calls `clearUpdateCallbacks()` before releasing the monitor, both on
reload and display destruction. In MoveIt 2.5.10 this method shares `update_lock_`
with `triggerSceneUpdateEvent()`: it waits for in-flight callbacks and prevents
later scene messages from calling the display. A null-pointer check alone would
not synchronize the two threads.

Sources: [display](https://github.com/moveit/moveit2/blob/2.5.10/moveit_ros/visualization/planning_scene_rviz_plugin/src/planning_scene_display.cpp),
[monitor](https://github.com/moveit/moveit2/blob/2.5.10/moveit_ros/planning/planning_scene_monitor/src/planning_scene_monitor.cpp).
The upstream source retains its BSD license. No robot model, scene topic,
collision policy, or lidar setting changes are included.

## Build an isolated overlay

Use a fresh terminal with ROS Humble's installed MoveIt **2.5.10**. Do not apply
this version-specific patch to a different release without reviewing it.
These commands build locally; they do not replace `/opt/ros` or start ROS nodes.
Run them from the `fault_detector_spot` checkout. Choose a new build directory
if the example directory already contains an unrelated build.

```bash
set -e
source /opt/ros/humble/setup.bash
patch_dir="$PWD/patches/moveit-2.5.10"
fix_dir=/tmp/rviz-scene-fix
mkdir -p "$fix_dir"
curl -fL https://codeload.github.com/moveit/moveit2/tar.gz/refs/tags/2.5.10 \
  -o "$fix_dir/moveit2-2.5.10.tar.gz"
(
  set -e
  cd "$fix_dir"
  echo 'f8606f3ead034f989e9adccc98943eeddb1d3eb09d18f7a4092534456b5f5ae0  moveit2-2.5.10.tar.gz' | sha256sum -c -
  tar -xzf moveit2-2.5.10.tar.gz
  patch --batch --fuzz=0 -d moveit2-2.5.10 -p1 < "$patch_dir/rviz-scene-monitor-lifetime.patch"
)
python3 "$patch_dir/check_lifetime.py" "$fix_dir"
CMAKE_BUILD_PARALLEL_LEVEL=4 colcon --log-base "$fix_dir/log" build \
  --base-paths "$fix_dir/moveit2-2.5.10/moveit_ros/visualization" \
  --packages-select moveit_ros_visualization --allow-overriding moveit_ros_visualization \
  --build-base "$fix_dir/build" --install-base "$fix_dir/install" \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF
```

`check_lifetime.py` compiles the actual patched cleanup method and upstream
callback-lock methods into a small C++ concurrency harness, with stand-ins for
the surrounding ROS/Qt objects. It exercises 100 controlled update/cleanup
interleavings, late updates and empty cleanup, and checks that the unpatched
negative control fails. It starts no nodes. This verifies callback ordering;
it does not replace a live RViz startup/reload test.

## Use on the next RViz launch

After sourcing the usual Spot workspace, source the overlay **last**, in the
terminal used for RViz:

```bash
source /tmp/rviz-scene-fix/install/local_setup.bash
ros2 pkg prefix moveit_ros_visualization
rviz2 -d "$(ros2 pkg prefix fault_detector_spot)/share/fault_detector_spot/config/arm_collision.rviz"
```

The prefix must point into `/tmp/rviz-scene-fix/install`. Only processes launched
from that terminal inherit the overlay; an already-running RViz is unchanged.
The application and MoveIt server need no restart for this plugin fix.
Use a fresh terminal without this overlay to revert. `/tmp` may be removed at
reboot; rebuild in a persistent directory if the fix needs to survive it.
