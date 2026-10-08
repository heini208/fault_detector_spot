# Fault Detector Spot – ROS 2 Behaviour-Tree Control for Boston Dynamics Spot

This repository contains the ROS 2 implementation of the **Fault Detector Spot** system:  
a behaviour‑tree–based control stack for the Boston Dynamics Spot robot, combining:

- High‑level base and manipulator control
- AprilTag‑based perception
- RGB‑D SLAM (RTAB‑Map) and Nav2 navigation
- Command recording & playback
- A PyQt5 GUI for development and experimentation

The system was developed as part of a **Master’s project at Hochschule Bonn‑Rhein‑Sieg**,  
in cooperation with **Fraunhofer IAO**, by **Marcel Stemmeler**.

For the full technical description, refer to the accompanying system design document:  
[System_Design.md](documentation%2FSystem_Design.md).

---

## 1. Repository Structure (high level)

- `fault_detector_spot/`
  - `fault_detector_spot/`
    - `behaviour_tree/`
      - `bt_runner.py` – main behaviour tree node
      - `nodes/…` – custom BT behaviours for sensing, mapping, navigation, manipulation, utility
      - `commands/…` – internal command classes and `CommandID` definitions
      - `ui_classes/…` – UI and recording control nodes
- `launch/`
  - `fault_detector_launch.py` – main launch file for real robot
  - `sim_fault_detector_launch.py` – simplified launch for simulation
  - `nav2_spot_launch.py` – Nav2 bringup tuned for Spot
  - `rtab_mapping_launch.py` – RTAB‑Map launch (mapping & localization)
- `config/`
  - `nav2_spot_params.yaml` / `nav2_sim_params.yaml` – Nav2 configuration
  - `mapping.rviz` – RViz config for mapping/navigation
  - `my_tags.yaml`, `my_tags_sim.yaml` – AprilTag configuration for `apriltag_ros`
- `documentation/`
  - `System_Design.md` – detailed system design & architecture (you pasted the latest version)
  - Additional docs, figures and example recordings under `images/System_Design/…`

---

## 2. Core Dependencies

### 2.1 ROS 2 & Robot

- **ROS 2 Humble Hawksbill** (recommended)
- **Boston Dynamics Spot** with:
  - Spot SDK 5.0.1 (via `spot_ros2`)
  - Optional manipulator arm (required for manipulation functions)
  - Body cameras (required), hand camera recommended

### 2.2 Packages from this ecosystem

You must have the following ROS 2 packages installed and sourced:

- **Robot & Messages**
  - [`spot_ros2`](https://github.com/bdaiinstitute/spot_ros2) – official Spot ROS 2 driver
  - `spot_msgs`, `bosdyn_msgs`, `spot_wrapper`, `spot_common`, `synchros2` (as required by `spot_ros2`)
  - [`fault_detector_msgs`](https://github.com/heini208/fault_detector_msgs) – custom message definitions for this system

- **Behaviour Trees**
  - [`py_trees`](https://github.com/splintered-reality/py_trees)
  - [`py_trees_ros`](https://github.com/splintered-reality/py_trees_ros)
  - `py_trees_ros_interfaces`

- **Perception**
  - [`apriltag_ros`](https://github.com/christianrauch/apriltag_ros) (or [AprilRobotics/apriltag_ros](https://github.com/AprilRobotics/apriltag_ros))
  - `pointcloud_to_laserscan` (if using the Nav2 + synthetic scan pipeline)

- **Mapping & Navigation**
  - [`rtabmap_ros`](https://github.com/introlab/rtabmap_ros) (and `rtabmap_slam`, `rtabmap_sync`)
  - [`nav2_bringup`](https://github.com/ros-planning/navigation2) and full Nav2 stack

- **UI & Tools**
  - `PyQt5` (Python package) – for GUI
  - `rviz2` – visualization

Python dependencies (partial):

```bash
pip install PyQt5 psutil
```

ROS dependencies are declared in [`package.xml`](package.xml); use `rosdep` to install what’s missing:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

---

## 3. Building the Package

Assuming a ROS 2 workspace `~/ros2_ws`:

```bash
cd ~/ros2_ws/src
git clone https://github.com/heini208/fault_detector_spot.git
git clone https://github.com/heini208/fault_detector_msgs.git
# plus spot_ros2, rtabmap_ros, nav2, etc., if not already in your workspace

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

Make sure the Spot SDK and `spot_ros2` setup instructions are followed as described in the [`spot_ros2` README](https://github.com/bdaiinstitute/spot_ros2).

---

## 4. Launching the System

### 4.1 Real robot (Fault Detector Spot stack)

The **primary launch file** is `fault_detector_launch.py`, which starts:

- `fault_detector_ui` – PyQt5 GUI
- `bt_runner` – main behaviour tree node
- `apriltag_node` – AprilTag detection for hand camera (`apriltag_ros`)
- `tag_observation_node` – tag fusion, TF resolution, and state publishing
- `move_close_to_surface_node` – force-guarded surface approach action server
- `record_manager` – command recording & playback node
- `micro_ros_agent` – UDP bridge for ESP32 micro-ROS sensor mounts
- `sensor_head_connection` – discovers acquisition endpoints and matches their
  IDs to the selected physical sensor mount

From your ROS 2 workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/Projects/spot/spot_sensor/microros_ws/install/local_setup.bash
source install/setup.bash

ros2 launch fault_detector_spot fault_detector_launch.py
```

The Agent defaults to UDP/IPv4 port `8888`, matching the sensor-mount firmware.
Its transport, port, and log level are configurable launch arguments:

```bash
ros2 launch fault_detector_spot fault_detector_launch.py \
  micro_ros_agent_port:=8888 micro_ros_agent_verbosity:=4
```

The status overview shows a green/red Agent indicator while keeping the
advertised endpoint hidden by default. Click `Show IP` to reveal `IPv4:port`;
the adjacent `Copy` button copies the complete ESP32 serial command, for example
`set-agent 192.168.178.69 8888`. Click the endpoint again to hide it. The address
is selected from the host's default IPv4 route. On hosts with multiple routes
(for example a VPN), override it explicitly:

```bash
ros2 launch fault_detector_spot fault_detector_launch.py \
  micro_ros_agent_address:=192.168.178.69
```

To run an Agent separately for debugging, disable the managed process so two
Agents do not compete for the same UDP port:

```bash
ros2 launch fault_detector_spot fault_detector_launch.py \
  launch_micro_ros_agent:=false
```

The hardware row reports physical attachment and network connection separately.
The attachment remains the confirmed source of hand-to-probe geometry. The host
discovers a head from its exact typed acquisition service. Each ESP32 enables
Micro XRCE-DDS hard liveliness, so the Agent removes the service when that
client stops responding. A different connected ID is shown as a mismatch and
is never substituted automatically.

The **Sensor Mounts** tab lists connected head IDs above the editable Mount ID
field. `Use ID` copies the selected detected ID into the form, avoiding manual
transcription. Detection is optional: users can still type, save, select, and
physically confirm an offline sensor definition. Connection state does not gate
arm movement and does not modify physical attachment state.

Each mount definition also owns zero or more generic acquisition channels. A
channel has a stable channel ID and an explicit source kind. ROS-topic channels
store a complete topic and `package/msg/Type`; the derived `spot_geometry`
source is recorded directly without pretending to be a ROS topic. New mount
forms include a removable `spot_geometry` channel by default. The setup form
polls the live ROS graph and offers current topics and advertised message types
as editable suggestions. Manual topics and types remain valid for offline
sensors. Mounts with no channels remain valid for geometry and movement.

Generic measurement persistence now uses one JSONL file per configured channel
under
`measurements/<object>/<routine>/<probe-point>/<UTC-date>/<start-timestamp>/`.
Independent recordings from the header instead use
`measurements/manual/<UTC-date>/<start-timestamp>/`.
Trailing context levels are optional, so object-only and object/routine
recordings omit the missing directories.
That recording directory contains `metadata.json` and one
`<channel-id>.jsonl` file per channel. The sidecar preserves the sensor ID,
channel snapshot, attachment revision, lifecycle state, and sample counts.
Recording directories and files are created exclusively and are never silently
overwritten.

ROS-topic channels are activated only for an open measurement. Their configured
`package/msg/Type` is resolved at runtime, and each received message is converted
to JSON with both its optional `header.stamp` source time and the host receipt
time. The derived Spot-geometry source likewise exists only during a recording.
It samples at a configurable rate (10 Hz by default) and stores the frozen
object pose together with live body, hand, and probe poses in `odom`, plus the
probe pose expressed relative to the inspection object. It writes directly to
its channel file and does not introduce a synthetic ROS topic.

`SensorAcquisitionCoordinator` is the single runtime owner of this process.
For physical channels under `/sensors/<sensor-id>/...`, it creates the local
subscriptions before enabling the matching ESP32 service and reports
`RECORDING` only after both the service acknowledgement and the first physical
sample arrive. Unrelated channels such as `/odom` cannot satisfy that readiness
condition. Stop first closes the local inputs, then disables the head, waits for
its acknowledgement, and finalizes metadata. A missing physical head is an
immediate skipped success and creates no empty measurement directory.

All lifecycle transitions (`IDLE`, `STARTING`, `RECORDING`, `STOPPING`, and
`FAILED`) are published latched on
`fault_detector/application/sensor_acquisition_state`. This is the state the
header's Sensor section consumes for its Record / Stop control; the UI does not
maintain a private recording flag. The same control submits the
recordable `start_sensor_recording` and `stop_sensor_recording` semantic
commands used during saved-workflow playback. A neighboring Folder button opens
the configured measurement root in the desktop file manager. Geometry TF
subscriptions and the startup watchdog are allocated only for an active
applicable recording. The launch file exposes only the measurement root;
timeout and sampling defaults stay local until there is a demonstrated need to
configure them.

Requirements:

- Spot is powered on and connected to the ROS machine (via `spot_ros2` configuration).
- `fault_detector_msgs` and `spot_ros2` are built and sourced.
- The `microros_ws` overlay containing `micro_ros_agent` is built and sourced.
- AprilTag config file (e.g. `config/my_tags.yaml`) matches your tags in the environment.

### Optional environmental collision checks for arm planning

The **Environment collision checking** control starts **Disabled** on every
application startup. It can be switched on or off whenever the command backend
is available, independently of mapping, localization, or lidar attachment.
Starting or stopping mapping does not change the setting.

The setting controls occupancy collision checks in MoveIt's current planning
scene. The application launch configures MoveIt's native point-cloud updater for
`/velodyne/points_sensor`, using
[`config/moveit_sensors.yaml`](config/moveit_sensors.yaml). The application starts
one shared lidar adapter when collision checking or mapping/localization needs
it, and stops it when neither needs it. Verify its mount calibration before use
(see below). RTAB-Map is not queried or imported for arm planning.

The updater uses 5 cm voxels, a 3 m sensing range and input capped at 5 Hz by the
adapter's monotonic clock. MoveIt's own ROS-time throttle is disabled so backward
clock jumps during recording replay do not stall updates. MoveIt's robot/attached-body
self-filter and free-space rays process this input. The sensing mask uses
`padding_offset: 0.20` and `padding_scale: 1.1` to exclude more returns around
the robot as its joints move. These settings apply to the whole robot model,
including legs; they do not enlarge the planning collision geometry or alter
motion targets. Native mesh padding expands vertices radially, so this is not
a uniform 20 cm shell. Nearby real obstacle returns can also be excluded.

Offline native-library checks used two captured poses and deliberately offset
their mask geometry by up to 5 cm. The larger mask removed robot/voxel overlaps
in all 416 tested combinations of offset and voxel-grid alignment; the previous
10 cm/1.0 mask had 16 arm-contact cases. This is a tolerance test, not evidence
that the live TF is wrong by 5 cm or proof for every moving-arm pose. Other poses
and unmodelled sensor heads still need validation.
The mask filters incoming scans; it does not erase the whole volume around the
arm or unobserved historical trails. Restart MoveIt to load changed settings
and reconstruct the scene from fresh observations. Clearing
the map alone does not reload this startup configuration; existing occupied
cells do not expire automatically.
It consumes the corrected lidar directly, without mapping's broad arm-exclusion
box. Sensor integration runs independently of arm requests and adds no map fetch
to a movement. The range limits each observation, not the accumulated map size.

**Integration prerequisites:** install `moveit_ros_perception` (Humble Debian
package `ros-humble-moveit-ros-perception`) from the same MoveIt release as its
libraries. Building this Python package alone does not supply or align those
dependencies. The configured lidar/OMPL path passes offline checks with MoveIt
2.5.10 and `moveit_msgs` 2.2.3, including native plugin loading. Keep the installed
MoveIt libraries consistent before starting sensing; package discovery alone
does not verify that a plugin can load.
The updated `spot_moveit_config` SRDF includes the standard floating virtual joint:

```xml
<virtual_joint name="odom_joint" type="floating" parent_frame="odom" child_link="body" />
```

That makes the scene frame `odom`, with the base pose supplied by existing
`odom` to `body` TF. The arm chain and targets remain relative to `body`.
MoveIt 2.5.9's planning-scene monitor chooses the robot model frame for native
occupancy; setting `octomap_frame` alone does not override a body-rooted model.
Against the old body-rooted model, accumulated obstacles would move with Spot.
The application launch therefore skips sensor configuration with a warning if
this joint or the perception package is missing, while keeping the existing arm
planner available. Rebuild `spot_moveit_config` along with this package so the
installed model includes the joint.

Enabling the control requests the adapter asynchronously; it does not start the
Spot driver or certify current obstacle coverage. Allow startup and inspect the
scene before planning. With no received data, the scene is empty. If data stops, stored occupancy stays;
there is no automatic expiry or fallback to disabled. Visible free-space rays
clear old observations over successive scans; occluded obstacles can remain.
This is an optional planning aid, not live collision monitoring during execution.
The existing execution guard stays active.

The application launch throttles one known console message: MoveIt's repeated
past-extrapolation warning for a stale hand-camera `tag36h11:<id>` frame relative
to `odom`. The first occurrence is shown, then at most one every 30 seconds.
Other warnings and errors remain visible, and native MoveIt file logs and
`/rosout` retain the full messages. This is console filtering, not a change to tag
timestamps, TF, collision checks or sensor freshness. MoveIt attempts to resolve
every known nonrobot TF frame on map updates; an unseen tag can outlive the
robot-transform history and cause this warning even when lidar TF is healthy.

Offline checks validate launch wiring, collision policy and the updated model's
body-relative geometry. They do not verify root-TF freshness, runtime planning
latency or endpoint accuracy with the floating root. Those need hardware checks
before relying on the new occupancy input.

After a MoveIt package update, use a fresh, normally sourced terminal and a normal
fresh application/MoveIt startup for the next authorized session. Do not reuse
processes from before the update: `GetCartesianPath` changed in `moveit_msgs`
2.2.3. Its added scaling fields keep their defaults; the executor continues to
control the requested movement duration.

Before each arm plan, the planner reads only MoveIt's Allowed Collision Matrix
and applies the requested occupancy policy. Disabled or explicitly bypassed
movements ignore the `<octomap>` object; enabled movements check it. Occupancy
itself is neither cleared nor replaced. Self-collision rules, explicit collision
objects, attached geometry, target tolerances, trajectory validation, and the
contact guard are unchanged. Cartesian requests retain `avoid_collisions=True`.
The policy preparation uses two asynchronous MoveIt service calls; it no longer
fetches or converts a map.

The checkbox **Ignore environmental obstacles for next basic arm movement**
applies to one submitted basic arm action, then clears immediately, including
rejected submissions. Wait, gripper, posture, and saved-workflow actions do not
consume it. Execution code can select the same bypass with
`ignore_environment_collisions=True`.

Safe and aligned pre-approach travel follows the global control at execution
time. Every **Move Close to Surface / Wall** command bypasses occupancy,
including positive stand-off distances. Final probe-point moves and their final
path waypoints always bypass it, both manually and through **Execute Probe
Point**. A bypass on the complete probe-point command does not disable checking
for its safe or aligned approach stages. Corrections, retries, contact retreat,
and returns along the final segment preserve that segment's bypass.

A toggle change affects subsequent planning and does not interrupt an executing
movement. Changing the setting during ordinary plan preparation invalidates that
pending movement; submit it again with the desired setting. Explicit bypass
movements are independent of the global setting. The UI confirms the backend's
setting and disables its toggle if status becomes stale or a change is pending.
Control availability describes the setting service, not sensor readiness.

MoveIt's collision matrix is shared. The application serializes policy updates
and plans; keep independent planning clients and collision-matrix writers idle
while it owns arm planning. A cancelled or timed-out policy update or plan must
finish remotely before another can start. If completion remains unknown, restart
the application and MoveIt together. An abandoned read-only scene request cannot
later apply its result. A policy-service failure rejects the movement rather than
silently changing its collision policy.

The retained command interfaces require `ignore_environment_collisions` in
`fault_detector_msgs`. When installing this change, stop the application before
replacing installed interfaces and rebuild these packages from the workspace root,
with the normal ROS, micro-ROS, and workspace dependencies sourced:

```bash
colcon build --symlink-install --packages-select fault_detector_msgs spot_moveit_config fault_detector_spot
source install/local_setup.bash
ros2 launch fault_detector_spot fault_detector_launch.py
```

Use a fresh terminal without an older collision experiment's `/tmp` overlay.
After restarting, verify **Disabled** without mapping, toggle on and off, and
confirm starting/stopping mapping leaves the selected setting unchanged. Ordinary
clear-space arm movements, the one-shot checkbox, and wall/probe bypasses can then
be checked in an authorized robot test. Environmental avoidance requires a
configured obstacle source and a separate planning-only validation first.
The implementation steps and retained sensor findings are in the
[collision avoidance plan](documentation/MoveIt_Collision_Avoidance_Implementation_Plan.md).
For the first authorized passive sensor check, the package includes
[`config/arm_collision.rviz`](config/arm_collision.rviz): an `odom`-fixed view of
the planning scene, robot collision geometry and corrected lidar, with optional
filtered points. It has no motion controls. The plan contains the launch command
and expected observations; this preset has only been checked offline so far.

### 4.2 Simulation / reduced setup

For a simplified, simulation‑oriented setup:

```bash
ros2 launch fault_detector_spot sim_fault_detector_launch.py
```

This launches:

- `fault_detector_ui`
- `record_manager`
- `sim_bt_runner` (instead of the full `bt_runner`)

You are expected to provide simulated topics for the UI and BT (e.g. via Gazebo or your own nodes).

### 4.3 Mapping and Localization (RTAB‑Map)

RTAB‑Map is launched isolated via [`lidar_rtab_mapping_launch.py`](launch/lidar_rtab_mapping_launch.py). This launch file:

- Uses the lidar cloud after the existing arm exclusion filter
- Starts `rtabmap_slam/rtabmap` in:

  - **mapping mode** (extend map) or
  - **localization‑only mode** (no map changes)

- Launches RViz with `config/mapping.rviz` for visualization

Mapping and localization started through the application now use the corrected
`/velodyne/points_sensor` source. The existing mapping arm-exclusion filter still
publishes `/velodyne/points_filtered` for RTAB-Map and lidar navigation. MoveIt
consumes the corrected source directly, with its own robot filter.

Example:

```bash
ros2 launch fault_detector_spot lidar_rtab_mapping_launch.py \
  db_path:=/path/to/your_map.db \
  delete_db:=false \
  extend_map:=true
```

This standalone mapping launch keeps its raw-topic default and does not own the
shared adapter. To use the corrected source outside the application, launch the
adapter separately and add `raw_lidar_topic:=/velodyne/points_sensor` above.

The lidar configuration preserves measured odometry height and tilt with
`RGBD/ForceOdom3DoF=false`, while keeping planar registration through
`Reg/Force3DoF=true`. Its existing height limits apply relative to the
gravity-aligned robot frame (`Grid/MapFrameProjection=false`), so a nonzero
odometry altitude does not move obstacles below the robot or filter them out.

After updating this launch, rebuild `fault_detector_spot` and restart mapping
through the usual controls to load the settings. **Create a fresh map under a new
name or unused database path** for validation. These settings do not repair poses
or grids already stored with height removed; extending an old map would mix the
two conventions. Keep existing maps intact. Saved-map localization across an
odometry reset needs separate validation before relying on that case.

See Section **10.5 Implementation Overview** and **10.6 Map lifecycle and process control** in [`System_Design.md`](System_Design.md) for the full flow.

The **lidar frame adapter** prepares a corrected sensor-origin cloud shared by
MoveIt's native updater and application-managed mapping/localization. It
converts `/velodyne/points` to `/velodyne/points_sensor` in
the physical `lidar_sensor` frame, preserving each acquisition timestamp. It
uses TF at that timestamp and the standard `tf2_sensor_msgs` transformation;
it does not estimate a mount from the cloud or change the Spot driver.

The behavior-tree helper owns one adapter runtime, using the existing mapping
runtime and collision preference as its demand sources:

| Mapping/localization active | Collision checking enabled | Adapter |
| --- | --- | --- |
| No | No | Stopped |
| Yes | No | Running |
| No | Yes | Running |
| Yes | Yes | Same single adapter |

Starts and stops run in a background worker. Map switches/save operations keep
the adapter alive; disabling just one consumer never stops the other's source.
Lifecycle checks use steady time, including during paused or rewound replay.
The application allows an initial two-second discovery window and reuses an
existing corrected-cloud publisher, even if no fresh scans arrive. It never
starts another adapter just because data is stale. Failed launches retry at most
every five seconds. Avoid simultaneous manual/application launches: ROS graph
discovery is not an atomic lock. Multiple publishers are reported as a warning.

An existing default adapter is stopped cooperatively through
`/fault_detector/lidar_adapter/stop` when both consumers are off. A rosbag or
unrelated publisher is not stopped. **After updating, stop any old manually
launched adapter once**; old processes do not offer this service. Subsequently,
normal application use needs no separate adapter terminal. The adapter launch
also stops its mount broadcaster when the adapter exits. Application shutdown
terminates adapter process groups it launched. For a manually started instance,
turn both consumers off before exiting the application so its shutdown service
can still be called; the app does not kill an external process group.

When hardware is available, first verify
[`config/lidar_mount_calibration.yaml`](config/lidar_mount_calibration.yaml)
against the current mount. Those values were captured previously from this
robot's SDK and have only been checked offline in this implementation. Then the
standalone launch is:

```bash
ros2 launch fault_detector_spot lidar_frame_adapter_launch.py
```

This starts only the adapter and the standard static-TF component. If a verified
physical lidar TF already exists, set `publish_mount_tf:=false` and
`sensor_frame:=` its actual frame name. `calibration_file`, `input_topic`,
`output_topic`, and `use_sim_time` are also launch arguments. Run only one
publisher for the chosen physical sensor frame.

The adapter uses queues of depth one, limits conversion attempts to 5 Hz and
100,000 points, and drops clouds older than 0.75 s or more than 50 ms in the
future. The age limit accommodates measured live lidar delays of about 0.4 s
with spikes to 0.56 s; it adds no waiting and does not change acquisition timestamps.
Freshness is checked both before and after conversion. Missing timestamped TF
drops that scan immediately; there is no wait,
latest-transform substitution, or stored scan to replay later. Malformed clouds
are rejected and warnings are throttled. Backward ROS clock jumps and clock-source
changes clear dynamic TF history.
Only the current driver's unorganized, packed little-endian XYZ32 format is
supported. The age, point-count and rate limits are read-only startup ROS
parameters on the adapter.

While the application is open, standalone passive adapter tests need mapping/
localization or collision checking enabled; with both off, the application
intentionally stops the adapter. This changes sensing only and sends no movement.
Before
enabling arm avoidance on hardware, inspect the MoveIt scene in RViz: verify the
scene frame is `odom`, geometry stays stationary during base motion, robot returns
are removed, and a moved visible obstacle clears. Then compare enabled and
bypassed planning-only requests before testing physical movement. Front and hand
cameras are not configured yet; this first input covers only what the rear lidar
sees.

### 4.4 Navigation (Nav2)

Nav2 is brought up isolated with [`nav2_spot_launch.py`](nav2_spot_launch.py). This:

- Includes `nav2_bringup/bringup_launch.py` with custom params
- Creates synthetic `/scan` topic from multiple depth cameras (`pointcloud_to_laserscan`)
- Starts the `nav2_cmd_vel_gate` node to coordinate Nav2 and Spot base control

Example:

```bash
ros2 launch fault_detector_spot nav2_spot_launch.py \
  use_sim_time:=false \
  map:=/path/to/your_map.yaml
```

The behaviour tree (`bt_runner`) interacts with Nav2 via the `NavigateToGoalPose` behaviour and Nav2’s `/navigate_to_pose` action.

---

## 5. Quick Start Checklist

1. **Hardware & network**
   - Spot online and reachable from your ROS machine
   - Spot time synchronized reasonably well with ROS machine (for TF/SLAM)

2. **Software**
   - ROS 2 Humble environment sourced
   - `spot_ros2` working (you can command Spot via its own examples)
   - `fault_detector_msgs` and `fault_detector_spot` built successfully
   - `rtabmap_ros`, `nav2` and `apriltag_ros` installed

3. **Bring up the stack**
   - Start RTAB‑Map (if you want mapping/localization)
   - Start Nav2 (if you want navigation to waypoints)
   - Start the Fault Detector stack:

     ```bash
     ros2 launch fault_detector_spot fault_detector_launch.py
     ```

4. **Use the UI**
   - Send simple commands (e.g. `STAND_UP`, `READY_ARM`)
   - Create a map and waypoints
   - Move between waypoints and to tags
   - Record and replay a sequence

For detailed behaviour descriptions and design rationale, always refer back to  
[System_Design.md](documentation%2FSystem_Design.md), [detailed_command_descriptions.md](documentation%2Fdetailed_command_descriptions.md) and the [`fault_detector_msgs`](https://github.com/heini208/fault_detector_msgs) message definitions.

## Arm-motion settings

[config/arm_motion.yaml](config/arm_motion.yaml) is the single source of defaults
for arm speed, ready offsets, settling, force baseline, contact detection,
retreat, and contact telemetry. Editing an existing value requires no Python
changes. The file is loaded once per process; edits apply on the next start.
ROS launch/parameter overrides still take precedence over the YAML defaults.
Explicit constructor arguments take precedence over both (useful in tests).

Arm movement result deadlines follow the submitted trajectory duration, with
`arm.motion.moveit_result_timeout_margin_sec` (15 seconds by default) added for
completion feedback and a minimum deadline of 30 seconds. This applies to both
MoveIt trajectories and direct Cartesian arm commands: a 50-second motion gets
at least 65 seconds to complete. Each motion has its own deadline; planning and
earlier workflow steps do not consume it. Cancellation, force freshness, contact
stops, and planning-response timeouts remain active.

To add a setting, add its `arm.*` key under `/**: ros__parameters` in the YAML,
then read it in the code that uses it:

```python
from fault_detector_spot.manipulation.arm_motion_parameters import ArmMotionParameters

config = ArmMotionParameters(node)  # omit node for standalone code
value = config.get("new_setting")  # reads arm.new_setting
```

Every YAML key is declared automatically on the node. There is no separate
parameter-name/default registry or resource-container entry to maintain. Keep
any physical constraints (such as positive distances) in the consuming code.
Missing keys and invalid value types raise errors instead of silently selecting
a Python fallback. Use YAML numbers and booleans, not quoted numeric strings.

Checkout execution reads the checkout's config; installed execution reads
`share/fault_detector_spot/config/arm_motion.yaml`. An installed copy must be
updated through the normal package installation workflow. Other configuration
files are unaffected by this arm-motion refactor.

### Body height

Select a body-height offset with the slider, then press **Change Height** to
apply it as a stationary stand command. Moving the slider alone sends no command.
The offset is relative to nominal standing height (−0.20 to +0.20 m).
Before relative/tag movement or dispatching a mapping waypoint to Nav2, the shared
base executor checks fresh `feet_center` → `body` TF height. It skips resetting
when the measured height matches its nominal reference within 1 cm. A changed
or unknown height triggers a zero-offset stand; movement waits for successful
stand completion, standing posture, and fresh height samples settled for 0.3 s.
The first preparation after executor startup establishes the nominal reference
from that completed stand, without assuming a fixed physical robot height.
Explicit height changes always require this reset, even if TF has not updated yet.

Missing/stale height feedback, reset rejection/failure, or confirmation timeout
blocks movement. Height is scoped to the stand command and never changes the
driver's persistent mobility parameters. The slider retains the selection for
reuse.

Application-managed walks finish with an explicit stationary pose hold. Relative and
tag moves first wait for measured arrival/settling, then replace the walking
command and verify the achieved position again using fresh post-stand samples.
The hold uses Spot's absolute `body_pose` stand command with the freshly measured
full `odom`-to-`body` pose, preserving achieved position, height, lean and yaw.
It does not request the default body alignment relative to the feet, which can
undo small turns accomplished by twisting the body. Missing or stale full pose
feedback fails the handoff without falling back to a recentering stand. Explicit
Stand and walking-height preparation retain their normal posture-reset behavior.
Tag correction uses observations captured after this final settling. This keeps
an old mobility trajectory from remaining active during subsequent arm work.
Normal walking obstacle avoidance remains enabled; standing still allows Spot's
normal balance adjustments. A failed stand or missing settling feedback prevents
successful completion and progression to the next queued command.

Waypoint execution owns its preparation in `WaypointNavigationExecutor`, so both
the waypoint tree and direct application callers must pass the same sequence:
confirm/stow the arm, prepare walking height, then dispatch the Nav2 goal. The
shared arm executor skips the stow command when fresh feedback already confirms
STOWED; otherwise it waits for stow completion and state confirmation. Arm state
is rechecked after height preparation and monitored during navigation. Loss of
stowed-arm confirmation requests Nav2 cancellation and fails the operation.
Preparation failures prevent Nav2 dispatch, and cancellation reaches the current
preparation or navigation operation, including goals accepted after cancellation.
After Nav2 reports success, the same base executor performs the stationary stand
and confirms fresh settling before waypoint execution reports success.
Nav2 goals sent straight to its action server by external clients still bypass
this application-owned preparation and completion.

Changes to this interface require rebuilding `fault_detector_msgs` together with
`fault_detector_spot` before launching the updated application.

### Runtime managers

`RtabmapRuntimeManager` and `Nav2RuntimeManager` inherit from the shared
`RuntimeManager` in `shared/ros/runtime_manager.py`. The parent owns the nested
ROS launch process, simulated-time propagation, process-group termination,
background-operation submission/polling, and retryable, idempotent shutdown.
`is_running()` reports process-group liveness; it does not claim that ROS nodes
are active or that navigation is ready. Runtime lifecycle management remains
separate from the arm, base, and waypoint movement executors.

RTAB-Map keeps its mapping/localization modes, database selection, active-map
publication, and save services. It uses one launch path for both modes and owns
its Nav2 runtime manager. The current API is `start_mapping()`,
`start_localization()`, `change_map()`, `stop(save=True)`, and `close()`;
`close()` stops without saving. Nav2 exposes `start()`, `stop()`, and `close()`.
Both expose `begin_runtime_operation()` and `poll_runtime_operation()` for
behavior-tree callers. Slow process termination does not hold the polling lock.

The old helper modules/classes and unused path/pose aliases, standalone save
wrapper, configuration setters, and process-only `wait_until_active()` were
removed. Internal imports and callers use the new runtime-manager names.

## Execute a saved probe point and record

In the saved probe-point controls, select the object, routine and point, set
**Recording duration** and **Retries**, then choose **Execute Probe Point and Record**.
The command moves through the routine's safe approach and the point's saved
pre-approach path to its aligned pose. Surface-relative points use their saved
wall distance; fully custom points follow their saved final probe path.
Recording starts with the selected object/routine/point context. The duration
begins when acquisition reports ready. Recording must stop and finalize before
the arm visits the reached waypoints in reverse and returns to the probe pose
captured before the first movement. These checkpoints use achieved poses in
odom; return motion does not depend on seeing the tag again. Motion between
checkpoints is guarded and planned normally, not a replay of joint trajectories.

Missing or stale tag observations get a five-second reacquisition window before
planning, each tag-relative motion step, and recording-context capture. No new
motion or recording starts during this wait. On timeout the command uses its
retry budget and checkpoint recovery, then waits for fresh observations again;
this does not perform an active camera search.

Retries are one shared budget across the combined command (zero means no retries).
If a movement fails, including collision/contact after a confirmed stop and local
retreat, the arm returns to its last successful checkpoint and retries only the
failed step. For example, failure on A → B → C recovers to B and retries C without
repeating A. The initial pose is the checkpoint if the first goal fails.
Return-path steps use the same retry policy and budget. Recording failures must
confirm acquisition shutdown, finalize partial data as failed, and begin a fresh
recording attempt; each attempt gets its full duration after acquisition is ready.

Success or exhausted forward/recording retries backtracks only reached checkpoints
to the safe approach pose, without returning to the pre-command arm pose.
The initial pose is used only to recover a failed first safe-approach move.
Checkpoint recovery and backtracking accept 20 mm position
error and 5 degrees orientation error; forward and measurement tolerances remain
independent. Unconfirmed stopping, failed local retreat, or failed
checkpoint recovery normally prohibits further motion. A verified checkpoint
position/orientation tolerance miss can retry that same recovery target after
confirmed stopping, consuming the shared retry budget. Collision or unsafe
failures during recovery remain terminal. If the return itself cannot be
completed within the remaining budget, execution stops at the last recoverable
checkpoint and reports failure; it never skips a blocked waypoint to go home.
Cancellation stops motion and acquisition without starting autonomous recovery.
Standalone move-to-tag path commands retain their existing behavior.

This uses `INTENT_EXECUTE_PROBE_POINT` with `duration_sec`, `retries`, `object_id`,
`routine_id`, and `probe_point_id`. Rebuild `fault_detector_msgs` together with
`fault_detector_spot` before using the updated UI and application/BT processes.
