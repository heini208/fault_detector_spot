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
  - `nav2_lidar_launch.py` – Nav2 bringup using the lidar configuration
  - `lidar_rtab_mapping_launch.py` – RTAB‑Map launch (mapping & localization)
  - `lidar_frame_adapter_launch.py` – standalone lidar frame adapter and mount TF
- `config/`
  - `nav2_lidar_params.yaml` – current Nav2 configuration
  - `mapping.rviz` – RViz config for mapping/navigation
  - `my_tags.yaml`, `my_tags_sim.yaml` – AprilTag configuration for `apriltag_ros`
- `documentation/`
  - `System_Design.md` – detailed system design & architecture (you pasted the latest version)
  - Additional docs, figures and example recordings under `images/System_Design/…`

---

## 2. Core Dependencies

### 2.1 ROS 2 & Robot

- **ROS 2 Humble Hawksbill on Ubuntu 22.04**, using the system Python 3.10
- **Boston Dynamics Spot** with:
  - Spot SDK 5.0.1 (via `spot_ros2`)
  - Optional manipulator arm (required for manipulation functions)
  - Body cameras (required), hand camera recommended

### 2.2 Packages from this ecosystem

You must have the following ROS 2 packages installed and sourced:

- **Robot & Messages**
  - [`spot_ros2`](https://github.com/bdaiinstitute/spot_ros2) – official Spot ROS 2 driver
  - `spot_msgs`, `synchros2`, `bosdyn_api_msgs`, `bosdyn_spot_api_msgs`
  - `spot_ros2` is the driver repository; its driver package is `spot_driver`.
    The workspace installer below builds the driver and its control packages.
  - [`fault_detector_msgs`](https://github.com/heini208/fault_detector_msgs) – custom message definitions for this system

- **Behaviour Trees**
  - [`py_trees`](https://github.com/splintered-reality/py_trees)
  - [`py_trees_ros`](https://github.com/splintered-reality/py_trees_ros)
  - `py_trees_ros_interfaces`

- **Perception**
  - [`apriltag_ros`](https://github.com/christianrauch/apriltag_ros) (or [AprilRobotics/apriltag_ros](https://github.com/AprilRobotics/apriltag_ros))

- **Mapping & Navigation**
  - [`rtabmap_ros`](https://github.com/introlab/rtabmap_ros): `rtabmap_slam`, `rtabmap_msgs`, `rtabmap_rviz_plugins`
  - [`nav2_bringup`](https://github.com/ros-planning/navigation2) and full Nav2 stack

- **UI & Tools**
  - `python3-pyqt5` – GUI, installed through rosdep/apt
  - `python3-numpy`, `python3-scipy`, `python3-yaml`, `python3-psutil`
  - `rviz2` – visualization

The retained launches also require `spot_moveit_config`, `spot_description`,
`moveit_ros_perception`, and `micro_ros_agent`. The workspace installer fetches
the project's matching `spot_moveit_config` and Humble micro-ROS Agent; cloning
only this repository does not install those dependencies.

### 2.3 Vendor dependencies and rosdep

The workspace installer performs these steps automatically. This subsection is
for manual builds or troubleshooting the vendor dependencies.

Use the [Spot driver installation instructions](https://github.com/rai-opensource/spot_ros2#installation)
for the SDK 5.0.1 driver revision, initialized submodules, and matching
[bosdyn_msgs 5.0.1 bundle](https://github.com/rai-opensource/bosdyn_msgs/releases/tag/5.0.1).
The SDK and generated ROS messages must use the same version. The application
imports `bosdyn.api`, `bosdyn.client`, and `bosdyn.util`; their standard rosdep
keys are `python3-bosdyn-api-pip`, `python3-bosdyn-client-pip`, and
`python3-bosdyn-core-pip`. Those keys are unversioned, so install the supported
versions before resolving the remaining dependencies:

```bash
sudo -H /usr/bin/python3 -m pip install -r src/fault_detector_spot/requirements-humble-sdk.txt
```

This is the documented vendor pip prerequisite. NumPy, SciPy, PyQt5, PyYAML,
and psutil use Ubuntu packages through [`package.xml`](package.xml).
The vendor SDK must be available to system Python, matching the driver's
installer; the offline check deliberately excludes user-site packages.
Do not treat an empty local rosdep rule for a vendor package as installation
evidence: check that `bosdyn_api_msgs` and `bosdyn_spot_api_msgs` are present in
the ament index and match SDK 5.0.1. The installation check below verifies both.

Source the dependency workspace before resolving this package's dependencies.
Provide any unresolved source package in that workspace; do not ignore rosdep
errors or add empty rules to hide missing packages.

```bash
rosdep install --from-paths src --ignore-src --rosdistro humble -y
```

`lifecycle_msgs` and `pointcloud_to_laserscan` have no direct consumers in the
retained application/launches. The former remains a dependency of ROS launch
internals; the retired camera Nav2 launch is not part of this package.

Driver-contract tests additionally use the `spot_wrapper` Python project from
the driver's initialized submodule. Install that project as described by the
driver instructions when running the full offline test suite.

---

## 3. Building the Package

On **Ubuntu 22.04 amd64**, create a workspace, download the installer into it,
and run it as your normal user (it uses `sudo` for system packages):

```bash
mkdir -p ~/ros2_ws
cd ~/ros2_ws
curl -fL https://raw.githubusercontent.com/heini208/fault_detector_spot/main/install.sh -o install.sh
bash install.sh
source ~/ros2_ws/setup.bash
```

If `curl` is unavailable, download `install.sh` through a browser into the workspace.
The script checks that Humble can be sourced and `rclpy` imported. If that fails,
it installs/repairs Humble using the [official ROS apt setup](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
It fetches the application and [pinned source dependencies](ci/humble.repos),
installs the SDK 5.0.1 bundles and ROS/Python dependencies, then builds the
messages, application, full Spot driver, MoveIt configuration and micro-ROS Agent.
Nav2, RTAB-Map and AprilTag packages are installed through rosdep.

Three hotfixes are applied before their affected packages are installed/built:
Spot wrapper posture feedback, the driver's completed point-cloud callback,
and the MoveIt RViz scene-monitor lifetime fix. The RViz fix requires MoveIt
**2.5.10** and builds a persistent overlay under `hotfixes/`; another installed
MoveIt version produces an explicit error instead of applying an unreviewed patch.
`setup.bash` sources this overlay last. SDK/source checksums, source pins and
build logs are retained under `evidence/` and `downloads/`.

The installer can resume after a failure. Existing source checkouts and the
driver configuration are preserved; conflicting patches or unexpected source
revisions stop the installation. It does not pull/reset existing checkouts,
modify shell startup files, start ROS nodes, or connect to Spot. Its final checks
import the installed application and driver and verify package/overlay locations.
Offline installer tests use simulated package/build commands; a complete fresh
machine installation remains an integration check.

After success, the script prints the required manual steps and creates
`~/ros2_ws/config/spot_ros.yaml` from the driver's example if it does not exist.
Edit `ros__parameters.username`, `password`, and `hostname` (Spot's IP address).
Enable `use_velodyne` if using the EAP lidar and check network access to Spot.
In each terminal, source `~/ros2_ws/setup.bash`. Start the driver with
`ros2 launch spot_driver spot_driver.launch.py config_file:=$HOME/ros2_ws/config/spot_ros.yaml`,
then start `ros2 launch fault_detector_spot fault_detector_launch.py` in another
sourced terminal when ready. Sensor calibration and robot-specific configuration
remain manual; installation does not validate them.

Both packages use version `0.0.0` while unreleased. Keep this package's version
and description synchronized in `package.xml` and `setup.py`; bump versions
deliberately for a release, not for each cleanup. This repository uses
[Apache-2.0](LICENSE), also declared in both metadata files and installed under
`share/fault_detector_spot/LICENSE`.

### 3.1 Offline installation check

Run the check from outside the source tree, after sourcing the new install:

```bash
cd /tmp
python3 ~/ros2_ws/src/fault_detector_spot/scripts/check_install.py \
  --prefix ~/ros2_ws/install/fault_detector_spot \
  --messages-prefix ~/ros2_ws/install/fault_detector_msgs \
  --source-tree ~/ros2_ws/src/fault_detector_spot
```

It imports every installed console entry point without calling `main`, verifies
the generated interfaces and installed launch/configuration files, and rejects
the retired Nav2 launch/gate. It does not start nodes. Launch descriptions can
also be inspected without starting their processes:

```bash
ros2 launch fault_detector_spot fault_detector_launch.py --show-args
ros2 launch fault_detector_spot nav2_lidar_launch.py --show-args
ros2 launch fault_detector_spot lidar_rtab_mapping_launch.py --show-args
ros2 launch fault_detector_spot lidar_frame_adapter_launch.py --show-args
```

A new build directory validates packaging, but a dependency-only installation
claim additionally requires a fresh Ubuntu 22.04/Humble environment provisioned
with the prerequisites above. An existing developer underlay can mask missing
dependencies and is not sufficient evidence for that claim.

### 3.2 Offline tests and settings isolation

`test/conftest.py` assigns a temporary `ROS_HOME` before collection and a fresh
runtime directory for each test, including its subprocesses. Tests use the
application defaults unless they explicitly save their own settings. Startup
settings caches are cleared between tests; the user's settings and runtime files
are not read or modified. The shared fixture does not import ROS or Qt.

After sourcing the required dependencies, run the offline suite with:

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q test \
  --ignore=test/test_preview_executor_responsiveness.py
```

The excluded preview integration test creates ROS nodes and is run separately
when node startup is authorized.

The automated lint gate checks production and launch code for Pyflakes (`F`)
and syntax (`E9`) errors using the declared `ament_flake8` dependency and the
project configuration in `setup.cfg`. It resolves source paths from the test
file, so generated `build`, `install`, and `log` trees are not scanned. This
matches the static-error scope of `ruff check --select F fault_detector_spot
launch`, with syntax checks included. Repository-wide formatting and docstring
style are deferred to a separate cleanup; the template PEP 257 gate is removed.

### 3.3 Automated baseline

The [baseline workflow](.github/workflows/baseline.yml) has two independent jobs:

- **Pure domain:** Python 3.10.12 in an isolated virtual environment, Ruff F,
  and the explicit [pure test list](ci/pure-tests.txt). The runner rejects ROS
  and Qt imports even if they are installed; forbidden imports fail rather than
  becoming skipped tests. It covers controllers, geometry, settings, persistence,
  and architecture rules.
- **Humble/Qt:** a fresh Ubuntu 22.04 container, ROS 2 Humble, pinned source
  dependencies, a copied build of `fault_detector_msgs` and this package,
  installed-entry-point/data checks, Ruff F, and the complete offline suite with
  Qt offscreen. It includes the real public-arm routing and stale UI feedback
  tests, as well as the dedicated force-guard and cancellation suites.

Source commits are pinned in [humble.repos](ci/humble.repos), vendor release
assets are checksum-verified, and Python dependencies are pinned in
`ci/requirements-*.txt`. The existing Spot wrapper posture fix is captured in
[spot-wrapper-posture.patch](ci/patches/spot-wrapper-posture.patch) against its
pinned upstream revision. The driver-contract tests require that patch; a local
uncommitted driver checkout is not a CI prerequisite. ROS/Ubuntu apt packages
remain distribution-resolved. Each run records their exact versions, Python
package locations, source revisions and working-tree hashes, including resolved
source dependencies from the micro-ROS build. Apt repositories can change;
these records make that drift visible rather than claiming a frozen apt image.

The jobs upload logs, JUnit results, build/import evidence and environment
manifests even on failure. The initial [baseline record](ci/baseline.json)
identifies the exact clean application commit and explicitly accounts for the
existing copyright-header skip and node-start integration exclusion. It was
measured on the developer's supported Humble environment. A first successful
hosted run is still required to validate fresh dependency provisioning; a local
build using existing dependencies does not establish that result.

Run the pure job locally without sourcing ROS:

```bash
python3 -m venv /tmp/fault-detector-pure
/tmp/fault-detector-pure/bin/python -I -m pip install -r ci/requirements-pure.txt
/tmp/fault-detector-pure/bin/python -I -m ruff check --select F fault_detector_spot launch
/tmp/fault-detector-pure/bin/python -I ci/run_pure_tests.py -q
```

After sourcing the declared Humble dependencies, run the build/import/offline
job using a new build directory. `CI_PYTHON` defaults to `/usr/bin/python3` and
can select an isolated Python 3.10 environment. Install
`ci/requirements-humble.txt` and the pinned, patched wrapper into that interpreter;
user-site packages are intentionally disabled. CI provisioning performs those
steps automatically:

```bash
bash ci/run_humble.sh ../fault_detector_msgs /tmp/fault-detector-build /tmp/fault-detector-results
```

`ci/provision_humble.sh` installs system packages and is intended for disposable
CI containers only. Neither job starts ROS nodes. The immediate-baseline fixture
in `test_arm_movement_executor.py` is limited to that module; dedicated baseline,
contact-policy and guarded-execution tests retain their real safety behavior.

### 3.4 Command liveness, uncertainty and stop confirmation

An acknowledgement timeout or uncertain submission leaves the original request
in `UNKNOWN` and retains its command lane. The controller queries the execution
owner through `fault_detector/_internal/commands/get_status`; it never resends
the motion. Late correlated feedback is still accepted. Missing owner history,
an unavailable query service, and a terminal status without `execution_idle`
confirmation all keep the lane occupied. Setup changes and playback cannot
continue through this uncertainty.

Software-stop acceptance means `STOP_REQUESTED`. The original request remains
associated with the lane, and `STOP_UNCONFIRMED` means the required stop evidence
is still missing. Only `STOP_CONFIRMED` from the execution owner releases the
execution barrier; the software-stop latch remains set until an explicit reset.
Each interrupted request must also be confirmed stopped. Stop
envelopes fence the affected request identities so delayed or duplicate delivery
cannot restart them. Duplicate normal requests return retained owner status
without running again. Recordings wait for confirmed outcomes; playback does
not retry or skip an unknown or unconfirmed request.

Fresh status-query replies also act as owner heartbeats after acknowledgement.
Each execution process has a unique instance ID, and each reply/publication has
an increasing sequence. The transport discovers the owner and its occupancy
before dispatch, binds requests to that instance, and rejects stale feedback.
Liveness uses local monotonic receipt time and steady timers; paused or jumping
ROS time does not extend the deadline. Sensor acquisition timestamps retain their
existing ROS-time validity rules.

`config/command_lifecycle.yaml` supplies positive finite `command.*` limits:
acknowledgement 3 s, heartbeat 5 s, reconciliation interval 1 s, query timeout
2 s, and polling period 0.1 s. The application API and UI load these parameters.
A healthy long operation can run indefinitely while queries keep answering.
Missing heartbeats enter `UNKNOWN` and reject new normal requests. Previously
queued work stays held; reconnecting to the same owner reconciles its retained
status without replaying motion. Repeated cancellation reuses its stop request
and remains latched, including after stopping has been confirmed.

A replacement execution process cannot certify that the previous process's
request finished. The original request stays bound to the previous instance,
and its late feedback is rejected after replacement is observed. A stop targeting
that previous instance reports `STOP_UNCONFIRMED` and blocks recovery motion.
There is no automatic reset or release based on an empty replacement ledger;
physical ownership must be resolved outside this protocol before a new session
can safely resume. This protocol covers execution-process restart and transient
client disconnection. Individual command history is not restored after an
application API process restart; the persistent shutdown markers described
below fence unresolved sessions. The UI retains pending work and displays
uncertainty when application updates disappear.

These states, execution identities/sequences, idle evidence, stop fences, status queries,
`SoftwareStopState` and `ResetSoftwareStop` are part of `fault_detector_msgs`.
Rebuild **both** `fault_detector_msgs` and
`fault_detector_spot` after updating these changes, then source the new install.
Old and new message installations must not be mixed.

### 3.5 Software stop, operator reset and arm recovery

The UI's **SOFTWARE STOP** latches application admission, cancels pending playback
and queued commands, and requests cancellation from the physical and sensor
acquisition owners. This application function is separate from the robot safety
E-stop. The retained `fault_detector/application/emergency_stop` service invokes
this software stop; acceptance confirms the request, not a stationary robot.
Stop admission does not wait for mapping shutdown or ordinary setup workflow
locks. The stop path does not stow the arm, close the gripper or clear its latch.

Base confirmation requires the owned Spot and Nav2 actions to settle, velocity
command sources to become quiet, and fresh stationary feedback. Publishing a zero
`Twist` alone is not stop confirmation. Missing feedback, unresolved owner
history, failed cancellation or unconfirmed acquisition shutdown keeps the
software stop latched and blocks motion. An interrupted measurement retains its
captured data and cancelled or failed metadata; it is not promoted to a successful
measurement, and reset does not restart acquisition.

The UI displays the authoritative aggregate on
`fault_detector/application/software_stop_state` (`SoftwareStopState`). The
latched topic carries the stop request ID, latch and confirmation flags, detail
and a sequence number. A **Reset software stop** request uses
`fault_detector/application/reset_software_stop` (`ResetSoftwareStop`) and is
admitted only after every owner confirms stopping. Reset is itself confirmed by
the execution owner before admission reopens. It requests no movement, discards
no unresolved ownership, and never replays cancelled work. A new stop during a
pending reset supersedes that reset.

**Recover arm (stow)** is a separate operator action after reset, submitted
through the normal command controller as `OperationalIntent.INTENT_RECOVER_ARM`.
It revalidates confirmed stopping, quiet base command sources, current confirmed
bare-hand attachment, a fresh measured arm pose and enabled environment collision
checks before stowing. Recovery with an attached sensor is rejected because
native stow clearance for that sensor is not established. Recovery does not close
the gripper and is excluded from recording/playback. The former **Reset State**
operation and intent value 6 are retired; callers must request reset and recovery
separately. The former zero-velocity-only stop intent (value 7) is also retired;
callers must use the confirmed software-stop protocol.

Shutdown closes command admission and pauses tree dispatch before requesting
arm, base, Nav2, setup and acquisition cancellation. The callback executor,
guard monitors, state subscriptions and stop clients remain available until
terminal action state and fresh physical-stop evidence are confirmed. The
confirmation deadline defaults to 10 seconds; independent cleanup has a further
20-second budget. Both are configured by `shutdown.confirmation_timeout_sec`
and `shutdown.cleanup_timeout_sec` in `config/command_lifecycle.yaml`.

A deadline reports **stop-unconfirmed** for each unresolved resource before
recordings, child runtimes and ROS resources are finalized. Failed cleanup
remains retryable. Unresolved ownership is retained in
`$ROS_HOME/fault_detector_spot/shutdown/<owner>.json` (default ROS home: `~/.ros`).
Each process also durably creates `<owner>.active` before admitting work and
removes its own marker only after confirmed shutdown and successful cleanup.
This keeps restart admission blocked even if writing the detailed timeout
record fails. Marker creation failure prevents admission.
The next startup exposes that uncertainty and blocks motion and software reset.
An empty new executor or a later successful shutdown cannot clear it. Recovery
requires independently verifying termination of the prior actions and fresh
physical stopping, then reconciling only those resource names through
`StopUnconfirmedStore.clear_confirmed()` before restarting; there is no automatic
reconciliation or UI reset bypass. An uncleared session marker is represented by
the resource name `active_session` and requires the same independent proof.

---

### 3.6 Persisted data and snapshot ownership

Command recordings saved by `RecordingRepository` contain `schema_version: 1`.
Files without a version (or with version `0`) use the legacy reader; other
versions fail explicitly. Loading never rewrites the source file. An explicit
save writes version 1, preserving execution retries, playback retries, skip
blocks, and command geometry. Migration fixtures live in `test/fixtures/recordings`.

The legacy migration retains the existing defaults for absent retry, collision,
walking-profile, body-height, tolerance, precision, and path-option fields. It
maps legacy `move_to_tag` commands with pre-approach offsets to
`follow_move_to_tag_path`. Missing playback retries and skip blocks default to
zero in both formats. Version 1 requires the complete command fields and does
not infer a different command from its payload. Transient attachment identity
remains excluded so playback binds the currently confirmed attachment.

AprilTag IDs and pixel coordinates require integers, excluding booleans;
numeric values must be finite numbers, and required names/string IDs must be text.
Numeric strings, fractional IDs, and values such as `null` converted to a name
are malformed input, not supported legacy variants. Existing optional defaults
in inspection and sensor definitions remain supported.

Writes retain the previous valid file until a complete temporary file is
synchronized and replaced. Partial writes and interrupted replacements fail
without publishing the temporary content. Measurement sample failures remain
latched in failed metadata; failed final metadata replacement can be retried.

Probe setup snapshots and motion attachment snapshots contain independent
copies of nested geometry. Their frozen outer dataclasses do not make every
nested object immutable. Consumers may modify their own copies; reservation
access also returns a copy, so those edits cannot alter retained server state.

## 4. Launching the System

### 4.1 Real robot (Fault Detector Spot stack)

The **primary launch file** is `fault_detector_launch.py`, which starts:

- `fault_detector_ui` – PyQt5 GUI
- `bt_runner` – main behaviour tree node
- `apriltag_node` – AprilTag detection for hand camera (`apriltag_ros`)
- `tag_observation_node` – tag fusion, TF resolution, and state publishing
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

**Import mount_creator ZIP** opens a file picker for `.sensorhead`
exports. It validates the collision mesh and its hand-relative placement, and
fills the hand-to-probe translation (metres) and rotation (degrees). Review or
adjust the tip values, then use **Save Sensor Mount**. The mount ID, name, and
acquisition channels stay with the current form. Invalid imports preserve the
existing draft. Unchanged tip fields retain the exported transform's precision.

Saving uploads the mesh in bounded 48 KiB chunks to the registry, validates it
again, and atomically stores the tip and geometry metadata in the sensor YAML.
Meshes live under `$ROS_HOME/fault_detector_spot/sensors/assets/<sha256>.stl`;
the original download is no longer needed. Existing assets are reused, and
editing a saved mount preserves its geometry without uploading the mesh again.
Upload failures leave the saved definition unchanged and keep the draft for retry.
Imports support the current hand reference template, metre units, and closed,
consistently wound binary STL meshes of at most 200,000 triangles and 32 MiB.

The small indicator below the import button is red for no geometry, amber for
imported geometry awaiting save, and green when opening a mount with saved
geometry. This indicator describes storage, not physical installation or MoveIt
synchronization. Select and confirm the physically installed mount as usual.

Before every MoveIt arm plan, the backend obtains the confirmed attachment,
fetches its saved mesh through bounded `GetSensorMesh` reads, and installs one
owned attached collision object, `fault_detector_sensor_head`. The exported
`hand_to_geometry` placement is applied once; the existing sensor-tip TF remains
independent. Missing geometry, pending confirmation, an incompatible robot model,
or a failed scene update blocks planning. Legacy mounts need a saved `.sensorhead`
package before they can be used for MoveIt arm motion.

The supported MoveIt model removes only the three forward stock-jaw collision
meshes (`front_jaw`, `middle_jaw`, `jaw_tooth`) and adds a fixed `sensor_head_slot`
at the `hand` origin. It retains the original wrist, rear housing and moving
finger. Confirmed **No sensor** restores those three stock meshes as the same
owned attachment. The retained rear housing is conservative where a physical
mount replaces more of the stock jaw.

The MoveIt model also includes closed convex hip meshes and a refined upper-arm
envelope derived from the supplied Spot visuals. `arm_link_hr0` owns the collision
geometry for both itself and its identity-fixed parent `arm_link_sh1`; the latter's
missing-collision warning does not indicate a separate unmodelled arm segment.
These model changes do not reduce the lidar filtering margins described below.

RViz's **MoveIt collision scene** display in `config/arm_collision.rviz` shows
the installed attachment in green after confirming the physical mount. The BT
runtime refreshes and verifies the scene while the arm is idle; no movement
request is needed. Importing and saving alone do not confirm an installation.
The application API, BT runtime, and MoveIt must be running: RViz itself does not
read the sensor registry. A movement requested during a refresh safely takes over
the existing sequence, then performs its own complete planning checks.
Use the planning-scene collision display; a plain RobotModel visual display still
shows the original driver visuals.

The collision viewer uses `spot_moveit_robot_description` and its matching
semantic-description topic, both published by MoveIt. This avoids receiving the
driver's different URDF on `/robot_description`, which would report unknown
`hand` / `sensor_head_slot` links and an empty arm group. Rebuild both packages
and restart MoveIt and RViz when updating this topic configuration. Robot opacity
is 70%; adjust **Scene Robot → Robot Alpha** to change transparency.

The **MoveIt collision scene** display starts disabled as a workaround for the
intermittent MoveIt 2.5.10 startup reload crash. Once RViz finishes loading, enable
its checkbox in **Displays** to show the robot and planning scene. This affects
only visualization, not backend collision checking. Avoid saving this preset with
the display enabled if you want to retain the workaround. It still needs live
validation and does not fix the plugin's race during later model reloads.

For the MoveIt 2.5.10 RViz crash in `onSceneMonitorReceivedUpdate()` during scene
monitor teardown, see the [isolated plugin patch and build instructions](patches/moveit-2.5.10/README.md).

This requires the one-time jaw-slot model change in `spot_moveit_config`, and a
rebuild of `fault_detector_msgs`, `fault_detector_spot`, and `spot_moveit_config`
before the next authorized application startup. The UI and backend must use
matching generated interfaces. Local offline tests do not deploy these changes.

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
application startup. Enabling requires a confirmed physical head attachment
(including confirmed **No sensor** for the stock jaw). The backend enforces this
requirement and the UI explains when confirmation is missing. Disabling remains
available; losing confirmation does not silently turn collision checking off.
The setting is independent of mapping, localization, or lidar attachment.
Starting or stopping mapping does not change the setting.

The setting controls occupancy collision checks in MoveIt's current planning
scene. The application launch configures MoveIt's native point-cloud updater for
`/velodyne/points_sensor_collision`, using
[`config/moveit_sensors.yaml`](config/moveit_sensors.yaml). The application starts
one shared lidar adapter when collision checking or mapping/localization needs
it, and stops it when neither needs it. Verify its mount calibration before use
(see below). RTAB-Map is not queried or imported for arm planning.

The updater uses 5 cm voxels, a 3 m sensing range and input capped at 5 Hz by the
adapter's monotonic clock. MoveIt's own ROS-time throttle is disabled so backward
clock jumps during recording replay do not stall updates. MoveIt's robot/attached-body
self-filter and free-space rays process this input. The sensing mask uses
`padding_offset: 0.15` and `padding_scale: 1.1` to exclude more returns around
the robot as its joints move. These settings apply to the whole robot model,
including legs; they do not enlarge the planning collision geometry or alter
motion targets. Native mesh padding expands vertices radially, so this is not
a uniform 15 cm shell. Nearby real obstacle returns can also be excluded.

Offline native-library checks used two captured poses and deliberately offset
their mask geometry by up to 5 cm. The former 20 cm/1.1 mask removed robot/voxel overlaps
in all 416 tested combinations of offset and voxel-grid alignment; the previous
10 cm/1.0 mask had 16 arm-contact cases. This is a tolerance test, not evidence
that the live TF is wrong by 5 cm or proof for every moving-arm pose. Other poses
and unmodelled sensor heads still need validation. The current 15 cm/1.1 mask is
an incremental reduction after the robot geometry and head-slot TF improvements;
the earlier results do not validate this smaller mask. Inspect robot returns and
nearby obstacles across arm poses before relying on it.
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
past-extrapolation warning for stale `tag36h11:<id>`, `fiducial_<id>` or
`filtered_fiducial_<id>` frames relative to `odom`. The first occurrence is shown,
then at most one every 30 seconds.
Matching ignores ROS console color codes while preserving the original output.
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

Before each arm plan, the planner reads the confirmed attachment and MoveIt's
robot description, then applies the head geometry and occupancy policy in one
scene diff. Disabled or explicitly bypassed movements ignore the `<octomap>`
object; enabled movements check it, including the installed sensor head. The
OctoMap itself is neither cleared nor replaced. Explicit world objects and
self-collision checking remain in force. Custom heads may touch only the hand
and its mounting wrist link; stock jaws retain the original adjacency allowances.
Cartesian requests retain `avoid_collisions=True`.

The planner verifies the applied mesh, placement, contact links, and collision
policy by reading the scene back. It repeats that verification after planning,
then queries the authoritative attachment again before accepting the trajectory.
The existing command/reservation guards prevent attachment changes while commands
are active or queued. Remote asset transfer is keyed by checksum, never a shared
filesystem path. Validated geometry is cached; scene synchronization still runs
for every plan, including after a MoveIt restart.

The checkbox **Ignore environmental obstacles for next basic arm movement**
applies to one submitted basic arm action, then clears immediately, including
rejected submissions. Wait, gripper, posture, and saved-workflow actions do not
consume it. Execution code can select the same bypass with
`ignore_environment_collisions=True`.

Safe and aligned pre-approach travel follows the global control at execution
time. The final approach segment of **Move Close to Surface / Wall** bypasses
occupancy, including positive stand-off distances. Its optional distance
observation excursion and return always retain environmental collision checks. Final probe-point moves and their final
path waypoints always bypass it, both manually and through **Execute Probe
Point**. A bypass on the complete probe-point command does not disable checking
for its safe or aligned approach stages. Corrections, retries, contact retreat,
and returns along the final segment preserve that segment's bypass.

Positive-distance **Move Close to Surface / Wall** first runs the shared
**Measure sensor tip distance** execution. Aligned pre-approach setup exposes
that same operation through **Test distance measure**, which reports distance
without performing the final approach. The test may move the arm and return;
its label reports both perpendicular wall distance and distance along probe +X.
A target distance of **0 mm** retains the force-guarded contact search and skips
depth observation entirely. In this mode, the command tolerance specifies the
requested backoff distance after contact (for example, 0.005 m requests 5 mm).
The existing guard caps backoff at the distance already travelled during the approach.
An omitted/zero tolerance falls back to `close_surface.tolerance_m`; the separate
`close_surface.contact_search_retreat_distance_m` setting has been removed.
Positive target distances still use tolerance as the allowed distance error.
The standalone distance test remains available.

During aligned pre-approach setup, **Orient sensor tip to surface** is available
after reaching the safe approach, before saving or reaching the final aligned
candidate. It runs the same
measurement and verified return, then aligns probe +X
with the measured target's inward surface normal. It preserves the returned
tip's final position in odom and uses the calibrated hand-to-probe transform
to keep the hand upright. The orientation movement requires collision-checked,
force-guarded Cartesian planning; the hand's interpolated path can cause a small
intermediate tip excursion. Fresh post-motion TF must confirm tip position
within 10 mm and axis alignment within 5 degrees before the achieved pose becomes
the candidate. **Save Final Aligned Pre-approach Pose** then becomes available;
orientation does not approve the pose automatically. Measurement failure, contact, changed attachment or
body reference prevents success; cancellation stops without starting another
movement. The separate check for tip movement between measurement and the start
of alignment remains 5 mm. The 10 mm final tolerance accepts positioning error;
it does not improve positioning accuracy or trigger a correction movement.
The existing action is now labeled **Orient camera to surface** and
continues to align the active probe using its lower-middle camera view;
sensor-tip orientation measures
the target along the original probe +X direction. The new button explicitly measures even when the
final approach is configured for force contact (0 mm).

Manipulation controls also expose **Orient sensor tip to surface** and **Test
surface distance** under **Surface actions**. Both use the shared measurement
operation independently of saved probe-point setup. **Move sensor tip close to
surface** measures the current distance before calculating an approach to the
requested final wall gap. None of these current-pose actions requires a
starting distance estimate: registered depth and the calibrated probe +X ray
locate the surface automatically. The distance test
returns the distance without making an approach movement. Surface orientation
remains a separate action; the approach rejects misalignment above its limit.

Measurement saves the original hand and probe poses in odom using fresh,
timestamped hand-depth TF. Setup and saved probe execution also supply their
aligned tag-relative goal for the return. That goal is resolved once into odom
before observation; later tag updates do not move the return target. It must
be within 20 mm and 5 degrees of the actual hand and tip before observation
can start. Standalone measurements keep the actual starting pose as their
return target. Depth near the forward probe ray seeds a local plane fit,
preferring nearer axial surfaces over a farther wall. Its actual intersection
with probe +X must have local unoccluded depth and lie within the fitted region.
A visible, supported target patch is measured directly. If it is outside the
image, a nearby fitted surface can guide one observation translation, with at
most 0.15 m extension from measured support. That preliminary plane is never
accepted as the final measurement. The translation brings the target just
inside the image with room for the
full plane-fit patch, preserving orientation and making no inward movement
along that saved axis. It does not move all the way to image center. An already
in-view target with missing depth is retried in place. After an excursion,
sparse depth frames are retried within the sampling timeout before returning;
the operation never searches farther blindly.
The translation is capped at 0.20 m and requires a complete, collision-checked,
force-guarded Cartesian path. It measures the original target, not the tip's
new location. Hidden-target extrapolation cannot produce a distance or normal
result; the moved camera must directly observe the actual intersection.

Registered depth can contain empty pixels between valid reprojected samples.
Target support therefore requires at least four measured returns enclosing the
exact target pixel in the local patch, rather than a fixed percentage of filled
image pixels. After fitting, at least 60% of the measured returns must agree
with the plane and those inliers must still enclose the target. Any local
foreground return more than the plane tolerance ahead of the fitted surface
rejects the measurement. No depth pixels are filled or invented.

After settling, the operation requires five distinct frames spanning one
second, at most 5 mm distance variation, consistent fitted normals, and a
newest valid reading no older than 0.5 seconds. Fits are reused only while their
camera calibration, timestamped TF, target and attachment remain unchanged.
The camera depth must be at least 0.290 m, and the measured axis intersection
must be 3–500 mm ahead. Measurements anchored to a saved tag target additionally
require agreement within 0.10 m of its expected wall distance. These are
correspondence/visibility limits, not promised measurement accuracy.

The observed plane is stored in odom. Original and returned poses require at
least three distinct acquisition timestamps spanning 0.3 seconds, with hand,
tip and body variation within 1 mm and 0.2 degrees. Images must have been
acquired after settling and be no older than 0.5 seconds; receiving an old
image recently does not make its pose fresh. Polling an already-consumed
image does not count another sample or erase the window when that cached
image ages between camera updates. New stale/invalid poses still invalidate
the window, and acquisition gaps above 0.5 seconds restart it. Waiting feedback
reports the stability limits and the last reset's pose variation or sample gap.
After an excursion, the arm
returns with collision checking and verifies hand and tip errors within 10 mm
and 2 degrees. A settled return outside those limits may authorize up to
three slow corrections to the same return hand pose in odom. A saved goal with
Precision enabled also retains its tighter selected tolerance. Each further
correction requires at least 10% improvement in the worst error relative to
its position/orientation tolerance. Both hand and tip errors must stay within
20 mm and 5 degrees per correction, with summed requested corrections capped
at 40 mm and 10 degrees. Corrections retain collision and force guards and run
at one quarter of the observation speeds (5 mm/s linear by default).

Before submitting an observation or return trajectory, execution rechecks
the odom-to-body conversion used for planning. A shift of the implied hand
target above 1 mm or 0.2 degrees discards the trajectory and repeats planning
and force-baseline acquisition, at most twice. Missing or stale transforms
prohibit dispatch. This does not continuously compensate body movement during
joint trajectory execution. Return diagnostics report hand, tip and body
displacements by odom axis. Missing transforms, changed reference/attachment,
contact/failure during return, or non-improving corrections prevent further
correction. The whole operation retains its 60-second deadline.

Tag and relative movements offer **Normal / Precision** selection in the main
manipulator tab. Aligned and custom final probe poses, and each path waypoint,
store their own precision setting. Normal preserves existing movement
behavior. Precision verifies a stable actual pose against the goal frozen in
odom and checks position and orientation after each correction. Tag and relative
movements allow corrections for position errors up to 200 mm, with summed
requested corrections capped at 400 mm. They retain the three-attempt limit,
required improvement, and angular limits used by measurement returns; observation
returns keep their smaller 20 mm / 40 mm translation limits. Collision and force guards
remain independent of this setting. Existing saved poses default to Normal.

The reported distance and final approach use the actual returned
tip pose. Body/reference or attachment changes invalidate the operation;
failed returns prohibit further autonomous recovery. Cancellation stops without
starting a return. A standalone test can report a tilted surface, but the final
approach still rejects axis misalignment above 5 degrees. Measurement and
approach share `close_surface.*` quality parameters; excursion limits include
`maximum_observation_translation_m`, `observation_speed_mps`,
`return_position_tolerance_m`, `return_orientation_tolerance_rad`, and
`maximum_measurement_duration_sec`. Fine-return parameters are
`return_correction_speed_scale`, `maximum_return_correction_translation_m`,
`maximum_return_correction_rotation_rad`, `maximum_return_corrections`,
`minimum_return_correction_improvement_fraction`, and the two
`maximum_return_correction_total_*` limits. Pose stability uses
`pose_stability_minimum_samples`, `pose_stability_minimum_span_sec`,
`pose_stability_position_tolerance_m`, and
`pose_stability_orientation_tolerance_rad`. Planning reference limits use
`planning_frame_position_tolerance_m` and
`planning_frame_orientation_tolerance_rad`.

Cartesian approaches use 2 mm path sampling and require at least 99.9% path
completion. MoveIt's relative joint-jump check remains enabled with a factor
of 3 against the whole-path average joint change; a factor of 2 can reject
smooth approaches whose initial wrist motion is larger than later increments.
A positive completion fraction with only the starting pose is reported as
probable first-transition jump rejection, rather than a confirmed IK failure.

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

This package does not supply a dedicated reduced simulation launch or simulation
BT runner. Simulation requires a separately configured environment that supplies
the interfaces used by the application.

### 4.3 Mapping and Localization (RTAB‑Map)

RTAB‑Map is launched isolated via [`lidar_rtab_mapping_launch.py`](launch/lidar_rtab_mapping_launch.py). This launch file:

- Uses the lidar cloud after the existing arm exclusion filter
- Starts `rtabmap_slam/rtabmap` in:

  - **mapping mode** (extend map) or
  - **localization‑only mode** (no map changes)

- Launches RViz with `config/mapping.rviz` for visualization

Mapping and localization started through the application now use the corrected
`/velodyne/points_sensor` source. The mapping arm-exclusion filter publishes
`/velodyne/points_filtered` for RTAB-Map and navigation clearing, plus
`/velodyne/points_obstacles` for navigation obstacle marking. MoveIt
consumes the adapter's separately rate-limited `/velodyne/points_sensor_collision`
output, with its own robot filter. Both streams reuse the same sensor-frame
conversion, so MoveIt's 5 Hz budget does not limit navigation's scan cadence.

Saving a waypoint starts an on-demand capture in either mapping or localization
mode. The action waits up to 10 seconds for usable localization acquired after
the Save request and a fresh current `map <- base_link` transform. It saves the
robot's current pose when capture completes, rather than its older scan-time
pose. Missing data reports progress while callbacks continue; cancellation,
timeout, or a map/clock change ends the capture without saving.

Both waypoint and landmark captures allow up to five seconds of localization
processing delay, configured by
`navigation.authoring.maximum_localization_delay_sec`. This allowance applies
only to scans acquired after the active Save request. It does not extend the
1.5-second limit on gaps between advancing processed results or on current robot
TF age. Pose timestamps and covariance checks are preserved, and pre-click
backlog, replayed estimates, unknown uncertainty, and missing transforms cannot
qualify a save. Finishing the capture removes its processing-delay allowance;
recovery and automatic refinement retain their existing freshness limits.

Saving a landmark works in both mapping and localization mode. Choose a tag in
the **Visible tag** dropdown beside **Add current pose as Waypoint**, then click
**Add Tag as Landmark**. The waypoint name field is only used for waypoints.
The dropdown follows currently visible tags and disables saving when none are
visible. Clicking the landmark button starts an on-demand capture of that tag;
it does not require a pre-filled
observation cache. The action waits up to 10 seconds and reports progress while
ROS callbacks continue receiving data. It requires localization and tag captures
acquired after the request, usable planar covariance, and three distinct, stable
observations of the tag in `map`. Acquisition delay is checked when a processed
estimate arrives; subsequent freshness checks measure the gap between advancing
estimates using both ROS and monotonic time. Normal RTAB-Map processing delay
therefore does not repeatedly erase the observation window, while stopped,
replayed or already stale estimates cannot qualify a save. The saved pose
retains position and orientation, using TF at each observation's acquisition
time. Captures wait briefly for delayed TF without erasing valid samples; saving
uses the newest qualified capture that is still fresh, even when a newer camera
message is waiting for its transform. Invalid localization, stale captures, and
changing tag poses clear qualification. A map or clock change aborts the capture.
Success, timeout, cancellation and closing setup discard the temporary samples;
idle authoring does not qualify tags continuously. A timeout reports which input
is still missing, without saving a partial landmark. Tags used as landmarks
must remain physically fixed; save them again from a known-good localization if
they move. Saved and deleted
landmarks are picked up by the recovery monitor without restarting localization.

Saved landmarks also receive small automatic refinements in localization mode.
The robot must be idle and stationary, with planar localization standard
deviation at most 10 cm and yaw standard deviation at most 3 degrees. Five
distinct tag captures and five matching, accepted RTAB-Map lidar-map
registrations must produce stable map-relative tag poses within 30 seconds.
Tags must be within 2.5 m; the burst must agree within 3 cm and 3 degrees.
The observed pose uses the lidar estimate and acquisition-time odometry,
including motion between the lidar scan and tag image. It does not use the
stored landmark to compute its own measurement.

Each refinement moves 10% toward the mean observation, capped at 1 cm and
0.5 degrees. An observation differing by more than 15 cm or 10 degrees is
rejected. Total adjustment is bounded to 5 cm and 3 degrees from the original
saved pose. Only one update is allowed per continuous sighting, and updates
must be at least five minutes apart; the tag must disappear for five seconds to
start another sighting. Moving out of range while still visible does not start
a new sighting. A qualified check that finds no adjustment necessary also
consumes the sighting, without writing metadata. The original pose, update count, last refinement time,
and observation stamp are preserved in optional `refinement` metadata alongside
the active `pose_map`. Existing map files remain readable. Atomic, locked
repository updates protect concurrent authoring and map deletion; stale
proposals are discarded. Refinement starts an attempt lasting at most 30 seconds for
one eligible tag at most once every five minutes, sampling at most twice per
second. Failed attempts also consume that interval; eligible tags take turns.
The first eligible attempt can start immediately. Tags outside usable range
do not consume an attempt. During the attempt, current odometry is checked
independently of slower lidar results; motion or missing odometry clears the
collected samples. Every geometry sample requires a distinct accepted result
matched within 1.5 seconds of its receipt. Gaps up to five seconds between such
results do not erase the burst. Refinement covariance, TF and geometry work is
skipped outside the bounded sampling attempt and its scheduled slots.

Refinement is deliberately disabled during mapping, because the map can change.
It is also blocked for the current map/runtime session after any `/initialpose`
seed, including landmark recovery and RViz resets, to avoid feeding a seed's bias
back into the landmarks. Metadata reloads and clock resets do not remove this
block. Attaching the monitor to an already running localization process also
blocks refinement because earlier pose seeds are unknown. A new localization
process requires fresh evidence again. Even repeated accepted lidar matches
can be biased, and RTAB-Map can restore its previous pose after restart: this is
a bounded refinement heuristic, not a guarantee of improved accuracy or a joint
pose/landmark optimizer. Original-anchor limits and persisted cooldowns still
apply after restart. Saving landmarks remains available in both modes.

Landmark recovery runs continuously in localization-only mode, including during
long commands. Five distinct, consistent tag captures establish a candidate
correction. Finite position standard deviation above 75 cm or yaw standard
deviation above 25 degrees can qualify for recovery, independently of the
stricter 30 cm / 10 degree saving and confirmation limits. These are uncertainty
estimates, not measured pose offsets. Finite uncertainty requires at least three
distinct estimates spanning ten seconds of receipt time, each with a matching
valid, unsuccessful lidar registration report. Successful lidar matches restart
this grace period. Missing, mismatched or malformed registration evidence cannot
qualify; briefly interleaved pose/Info callbacks can still complete a result.
An explicit RTAB-Map `9999` not-localized covariance report retains the shorter
three-estimate, two-second debounce without requiring registration Info.
Switching between these two paths starts a new evidence run. Missing, stale or
malformed poses cannot qualify. Confident localization always suppresses new resets and
retries, even when it disagrees with the saved landmark or a previous reset
remains unconfirmed. Recovery accepts processing delays up to five seconds and
gaps of up to five seconds between advancing estimates, so slower RTAB-Map
results do not continually reset the uncertainty count. Repeated timestamps
cannot extend that timeout or count as new evidence. Saving and refinement
retain their stricter 1.5-second receipt gate; tag and odometry freshness limits
also remain unchanged. Acquisition stamps are preserved for TF.
RTAB-Map treats `/initialpose` as a reset applied at its
next scan, so publication waits for idle command execution and one second of
fresh, nearly stationary odometry. Acquisition-time odometry accounts for motion
between the tag image and the current robot pose. Conflicting landmarks withhold
correction. This monitor does not stop an active navigation command.

A reset is followed by a two-second settling period, during which the existing
command manager keeps normal commands queued; emergency cancellation bypasses
this pause. This bounded pause covers nominal scan processing, but is not an
acknowledgment from RTAB-Map: severely delayed scans or external robot motion
still require deployment validation. Recovery needs three fresh,
accepted RTAB-Map registrations whose poses have usable covariance and agree with
the landmark. Attempts are at least 15 seconds apart, require new tag captures
and sustained low-confidence estimates acquired after the preceding reset,
and stop after three unsuccessful requests. The budget resets after confirmed
recovery, a map/runtime or clock change, or a changed set of landmark anchors.
The UI's persistent **Last landmark event** line and console logs report reset
requests, confirmed recovery, exhausted recovery, and successful refinements,
with the tag and map included in the UI. The line shows the latest event's time
and expandable details; a request explicitly says that confirmation is still pending.
Limits live in [`config/landmark_localization.yaml`](config/landmark_localization.yaml),
shared by the BT and application API. These initial values need validation with
recorded data for the installed sensors; covariance alone is not proof that
localization is correct.

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

RTAB-Map keeps only the latest waiting scan and uses best-effort scan delivery
with capture-time TF odometry. Its 2 Hz detection target limits processing rate;
it does not drain a queue of old scans. The bounded queues prevent avoidable
backlog without changing acquisition timestamps. Processing spikes can still
exceed the localization freshness limit; Save waits for an acceptable result
within its collection deadline. Routine RTAB-Map timing output is hidden by the
`warn` logger default; `rtabmap_log_level:=info` restores it for diagnosis.

The lidar configuration preserves measured odometry height and tilt with
`RGBD/ForceOdom3DoF=false`, while keeping planar registration through
`Reg/Force3DoF=true`. New maps created through the application record
`localization_frame=feet_center` in their metadata. RTAB-Map builds their occupancy
grid relative to this measured ground reference, with a 5 cm ground cutoff and
`Grid/MapFrameProjection=false`. Landmark recovery uses the same recorded pose
reference. Waypoints continue to describe `map <- base_link`.

Existing metadata without this field and databases without metadata retain
`base_link` and the previous 12 cm cutoff. Their reference cannot be changed in
place because RTAB-Map also stores scans relative to it. The standalone launch
retains those legacy defaults; for a **new, unused** database, pass
`frame_id:=feet_center ground_max_height:=0.05` to use the ground reference.

Both Nav2 costmaps mark lidar returns above 5 cm and up to 1.5 m relative to
acquisition-time `feet_center` TF. This removes the previous dependency on
absolute map/odom altitude and the local costmap's two-voxel marking requirement.
The selected cloud retains the actual sensor frame and timestamp; the full
self-filtered scan supplies clearing rays, including ground returns. The arm
exclusion box remains relative to `base_link`. Missing ground TF suppresses the
obstacle stream so Nav2's observation deadline detects missing data, rather than
receiving an empty cloud that could imply a clear scene. This navigation filter
works with both legacy and new maps.

Nav2 retains each source's latest scan and requires an update within 0.5 seconds,
covering the observed arrival jitter while detecting a stopped stream. It does
not purge a newly received scan merely because sensor delivery took longer than
the previous 0.3-second persistence window. The filter also rejects scans older
than 1 second at input or publication, including time spent waiting for TF;
receiving queued old data cannot keep the costmap falsely current.

The self-filter uses bulk ROS byte-array serialization for its output clouds,
avoiding Python validation of every byte. It checks obstacle age after
serialization too. Periodic filter diagnostics include input age and processing
time for the body transform, filtering and obstacle stages, to distinguish
upstream delay from local processing. Repeated Nav2 stale-buffer warnings are
limited on the main launch console to the first warning and one reminder every
five seconds per costmap/source. File and ROS logs retain every warning; the
costmap freshness checks remain active.

After an authorized rebuild and restart, **create a fresh map under a new name**
to validate the corrected saved occupancy grid. Existing databases and their
stored grids are not rewritten. Sensor coverage, the 5 cm cutoff, and saved-map
localization across an odometry reset still need hardware validation.

See Section **10.5 Implementation Overview** and **10.6 Map lifecycle and process control** in [`System_Design.md`](System_Design.md) for the full flow.

The **lidar frame adapter** prepares a corrected sensor-origin cloud shared by
MoveIt's native updater and application-managed mapping/localization. It
converts `/velodyne/points` to `/velodyne/points_sensor` for navigation and
`/velodyne/points_sensor_collision` for MoveIt in the physical `lidar_sensor`
frame, preserving each acquisition timestamp. It
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
`output_topic`, `collision_output_topic`, and `use_sim_time` are also launch
arguments. Run only one publisher for the chosen physical sensor frame.

The adapter uses queues of depth one and limits conversion attempts to 20 Hz and
100,000 points. The conversion limit leaves margin for the configured 10 Hz
driver's arrival jitter; navigation keeps its 0.5 s observation deadline.
MoveIt's output retains a separate 5 Hz monotonic rate cap, with no second
conversion or retained cloud backlog. The adapter drops clouds older than 0.75 s
or more than 50 ms in the future. The age limit accommodates measured live lidar delays of about 0.4 s
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

The application's `Nav2RuntimeManager` starts
[`nav2_lidar_launch.py`](launch/nav2_lidar_launch.py). This:

- Includes `nav2_bringup/navigation_launch.py` with
  [`nav2_lidar_params.yaml`](config/nav2_lidar_params.yaml)
- Uses RTAB‑Map's `/map` for the global costmap
- Uses `/velodyne/points_obstacles` for obstacle marking and
  `/velodyne/points_filtered` for clearing

Standalone launch, when the mapping/localization and lidar inputs are available:

```bash
ros2 launch fault_detector_spot nav2_lidar_launch.py use_sim_time:=false
```

The `params_file` launch argument can select a different Nav2 configuration.
The behaviour tree's `NavigateToGoalPose` behaviour delegates to
`WaypointNavigationExecutor`, which owns arm stow, walking-height preparation,
Nav2's `/navigate_to_pose` action, cancellation, and stationary completion.

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
for arm speed, the ready pose, settling, force baseline, contact detection,
retreat, and contact telemetry. Editing an existing value requires no Python
changes. The file is loaded once per process; edits apply on the next start.
ROS launch/parameter overrides still take precedence over the YAML defaults.
Explicit constructor arguments take precedence over both (useful in tests).

Ready Arm moves the hand to a fixed `body`-frame pose, including when already
deployed: `(x, y, z) = (0.60, 0.0, 0.36)` metres, with identity orientation
(hand facing body +X). This is an approximate replacement for the former
stowed-hand pose plus a 5 cm forward / 10 cm upward lift, not a measured
calibration. Adjust `arm.ready_position_x_m`, `arm.ready_position_y_m`, and
`arm.ready_position_z_m` in `arm_motion.yaml` as needed. The former relative
`arm.ready_forward_distance_m` and `arm.ready_lift_distance_m` settings are
replaced by these absolute coordinates. Movement uses the Ready Pose speed.

Every successful ready movement ends with a native `ArmStopCommand`, followed
by physical-stop confirmation and deployed-state verification. Automatic
preparation for another arm command uses the same pose when starting stowed.
Stow Arm first moves to this pose and completes the same stop verification,
then sends Spot's native stow command and verifies the stowed state. Failure
or cancellation before that transition prevents the native stow command.
An already-stowed arm stays stowed. The final native folding motion retains
Spot's own timing; the Ready Pose speed controls the preceding movement.

Guarded movements complete collision-scene preparation and MoveIt planning
before collecting a fresh force baseline. Only after that baseline succeeds
is the prepared trajectory submitted and the motion force watchdog activated.
Planning failures retain their MoveIt error code and cannot be masked by a
force timeout during planning. Collision-scene validity is checked again at
trajectory submission.

MoveIt pose planning allows up to 15 seconds to find a solution, configured by
`arm.motion.moveit_allowed_planning_time_sec` in `config/arm_motion.yaml`.
The motion-planning response deadline allows at least this budget plus two
seconds; scene and Cartesian service deadlines remain seven seconds. A larger
budget gives difficult goal sampling more time but cannot make an unreachable
or colliding candidate valid. Changes apply on the next application start.

If the force baseline is unstable before a guarded arm move, execution sends
one native `ArmStopCommand` to stop the previous arm command. After fresh
velocity feedback confirms the stop, it rebuilds the plan and collects a new
force baseline. The requested move starts only if that baseline passes the
existing limits. An unsuccessful retry, missing force data, or an unconfirmed
stop fails the movement; there is no repeated automatic recovery loop.

During cancellation, physical-stop monitoring continues after its initial
timeout, so later settling can release the arm for recovery. The original
action must also terminate before another motion starts. Emergency stow shows
the stop-confirmation detail while waiting.

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

### Guard diagnostic timing

The offline regression budget is **50 ms** for a force callback and for its
stop-request dispatch, matching the guard monitor's existing 50 ms period.
This budget was chosen before measurement. It covers local software dispatch;
it does not establish a deployed ROS scheduling bound or physical stopping time.

Raw contact logging uses one writer and a bounded queue (256 records by default).
Callbacks enqueue without waiting for storage; a full queue drops the incoming
record and increments `ArmContactTelemetry.diagnostics['dropped_samples']`.
File creation, JSON encoding, writes and flushes run in the writer. Storage
errors disable raw logging and retain `last_error`. Closing drains for at most
one second by default and reports a timeout while retaining the writer for a
later cleanup retry. Dropped diagnostics cannot be treated as a complete trace.

`GuardedProbeMonitor.diagnostics` returns a copied snapshot with the monitor
period, callback duration and delay, observation age, request/context IDs,
attachment revision, sensor ID, movement sequence, phase and stop-confirmation
state. Force delay/age use the locally received force timestamp, including any
wait for the execution lock; they do not measure sensor/network age. Timer delay
uses the expected callback time. `stop_requested_at` records local dispatch,
separately from stop acceptance and physical confirmation. These snapshots
remain available when raw logging is disabled or fails.

On 2026-11-02, five isolated trials per case used the real state source, monitor,
guard and telemetry with fake ROS interfaces. Each delayed-storage case injected
100 ms into file open, write or flush. Maximum times in milliseconds were:

| Storage case | Previous callback / stop dispatch | Current callback / stop dispatch |
| --- | ---: | ---: |
| Logging disabled | 0.257 / 0.249 | 0.274 / 0.188 |
| Logging enabled | 0.615 / 0.606 | 0.392 / 0.304 |
| Delayed open | 103.500 / 103.460 | 0.271 / 0.164 |
| Delayed write | 102.295 / 102.242 | 0.578 / 0.388 |
| Delayed flush | 102.594 / 102.530 | 0.482 / 0.382 |
| Failing open/write/flush (worst) | 0.593 / 0.585 | 0.364 / 0.220 |
| Observation exception | 0.150 / 0.142 | 0.312 / 0.269 |

The delayed cases exceeded the declared budget before decoupling storage.
Afterward, the maximum observed dispatch delay was 0.035 ms and observation age
was 0.046 ms. The tests in `test/test_guard_diagnostic_timing.py` also hold storage
blocked until after force-stop or cancellation dispatch, saturate the queue,
and inject diagnostic failures. No robot, ROS executor, or live node was started;
hardware and deployed-load timing remain separate validation work.

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
After Nav2 reports success, the executor waits for 0.3 s without `/cmd_vel`
messages before submitting the stationary stand. Even zero-velocity messages
from Nav2's smoother become new Spot commands and can otherwise override that
stand. A stream still active after 3 s fails the handoff; cancellation retains
ownership until the stream is quiet and fresh measured feedback proves stopped.
The same base executor then requires a successful stand and fresh settling before
waypoint execution reports success. Failed base commands include Spot's mobility
feedback status alongside the driver's message, distinguishing overrides,
timeouts, and robot-state failures.
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

Inspection's Routine Navigation shows the saved map's runtime status next to its
name: correct localization map, different localization map, no running map, or
mapping in progress. Unavailable runtime reports are shown separately so an old
map observation cannot appear current. The indicator compares the saved routine
assignment, independently of an unsaved dropdown selection.

**Launch Map** queues localization of the saved map. Matching localization is
reused; another localization map or any mapping session is stopped before the
target launches directly in localization mode. Mapping uses the existing save
and stop sequence. A failed stop prevents the replacement launch. The runtime
error is carried separately in `NavigationSetupState.runtime_error`, so a failed
waypoint-save request does not make a healthy map appear stopped.

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

### Basic UI settings

The **Settings** tab saves startup preferences, default manual arm/base modes,
normal/precision translation and rotation speeds, ready-pose speed, and the
normal arm force-policy multiplier. Save, reload, and reset-to-default controls
are provided. Reset marks this tab’s overrides for removal; click Save to apply the reset.

Settings are stored atomically in `$ROS_HOME/fault_detector_spot/basic_settings.yaml`
(default `~/.ros/fault_detector_spot/basic_settings.yaml`). The UI, application API,
and behavior-tree backend must share that directory. Restart all three to apply
changes; saving does not reconfigure running processes or start/restart nodes.
Saved speed settings override matching ROS/YAML defaults; explicit execution
speed overrides and per-command speed scales remain in effect. Saved probe-point
mode choices are not rewritten. Untouched fields remain inherited, including on an
unchanged Save. Editing the common base speed makes it apply to both tag and offset
moves; defaults preserve their separate existing speeds (100/150 mm/s for normal
offset/tag moves). Precision arm speed inherits the effective normal speed, including
ROS or explicit speed-policy overrides, unless independently configured. Zero in the
precision-speed YAML parameters means inherit; the UI displays the inherited default.

Settings documents now use version 2 and store only explicitly edited overrides.
Version 1 files stored full forms: default-valued entries are treated as inherited
when loaded, while non-default choices are retained. Reading does not rewrite the
file. Subsequent saves use version 2; explicit version 2 overrides remain explicit
even when equal to a factory value. Reset and Save both tabs to remove all overrides.

Startup mount approval assumes the remembered sensor (or bare hand) is physically
installed and uses the existing attachment controller to confirm it. Missing sensor
definitions are not approved. Startup environment checking waits for valid attachment
confirmation; subsequent manual toggles remain authoritative for the session.

Force threshold 100% preserves the configured minimum/reference/maximum policy.
Larger percentages reduce sensitivity; values must be positive and finite. This
multiplier does not change the hard-force ceiling or explicit thresholds used by
surface approach/contact search. Precision arm speeds configure the initial move;
existing precision verification and corrective speed reductions are retained.

### Advanced UI settings

**Settings → Advanced** adds arm/walking accuracy, correction limits, force thresholds,
retreat and telemetry controls, surface-approach tuning, navigation limits and arrival
accuracy, MoveIt scaling/obstacle-map settings, and tag observation stability. Both
settings tabs share the existing settings file; saving or resetting one tab preserves
the other tab's saved values. Reset remains unsaved until Save is clicked. The fields
use mm, degrees, seconds, newtons, and percentages and ignore mouse-wheel edits.

Arm position defaults populate new command and path-point editors; explicit and saved
point tolerances are retained. Separate turning defaults preserve the original
manual Precision tolerance (2 degrees) and probe setup tolerance (0.087 radians,
approximately 5 degrees). The arm correction limit caps
Precision retries and can disable the normal tag correction at zero; ordinary tag
moves retain their existing single-correction behavior. Fixed checkpoint and surface
orientation correction rules remain separate. Error, improvement, and travel limits
still bound Precision corrections regardless of the configured attempt count.

Advanced values apply on the next process start. Navigation overrides are applied by
the application's Nav2 runtime manager to both the controller and velocity smoother.
MoveIt obstacle-map overrides are applied by `fault_detector_launch.py` to `move_group`
only. Restart those processes as part of the application restart to apply these
changes. Generated parameter files live under the runtime directory in
`generated_settings/`; packaged YAML files are never rewritten by the settings page.
Standalone launches outside these application entry points retain their own parameters.
