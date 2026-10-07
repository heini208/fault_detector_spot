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

Example:

```bash
ros2 launch fault_detector_spot lidar_rtab_mapping_launch.py \
  db_path:=/path/to/your_map.db \
  delete_db:=false \
  extend_map:=true
```

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

Waypoint execution owns its preparation in `WaypointNavigationExecutor`, so both
the waypoint tree and direct application callers must pass the same sequence:
confirm/stow the arm, prepare walking height, then dispatch the Nav2 goal. The
shared arm executor skips the stow command when fresh feedback already confirms
STOWED; otherwise it waits for stow completion and state confirmation. Arm state
is rechecked after height preparation and monitored during navigation. Loss of
stowed-arm confirmation requests Nav2 cancellation and fails the operation.
Preparation failures prevent Nav2 dispatch, and cancellation reaches the current
preparation or navigation operation, including goals accepted after cancellation.
Nav2 goals sent straight to its action server by external clients still bypass
this application-owned preparation.

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
