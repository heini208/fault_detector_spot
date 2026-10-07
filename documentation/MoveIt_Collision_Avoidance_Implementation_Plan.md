# MoveIt collision avoidance implementation plan

Provide an optional environmental occupancy layer for arm planning. Preserve the
existing motion pipeline, endpoint accuracy, trajectory checks, and contact guard.
The RTAB-Map snapshot approach has been removed. Native MoveIt lidar input is
configured, with the model-frame prerequisite and hardware validation described
below. This document describes the current foundation and remaining work.

## 1. Independent control — implemented

- Start disabled on every application startup. The UI can toggle the preference
  whenever its command backend is available. Mapping and localization do not
  enable, disable, or gate it.
- `ArmCollisionControl`, owned by `RobotCommandResources`, is the single state
  owner. `HelperInitializer` initializes it before any arm planner is requested.
- Retain `fault_detector/set_arm_collision_checking` (`SetBool`). Publish the
  preference separately on `fault_detector/arm_collision_checking`
  (`DiagnosticStatus`), with a steady heartbeat for stale-status detection.
  Availability means the control is online; it does not certify sensor readiness.
- Retain the one-shot basic-movement checkbox and transported
  `ignore_environment_collisions` field. UI intents still pass through
  CommandController, ROS command transport, Behavior Tree, and execution owners.
- Safe and aligned approaches follow the global setting. Move Close to Surface /
  Wall and final probe-point segments always bypass environmental occupancy,
  including manual paths, Execute Probe Point, retries, and contact retreat.
  Self-collision checks and the execution guard remain active.

## 2. Small MoveIt policy adapter — implemented

Before planning, read only the Allowed Collision Matrix, apply a scene diff that
changes its `<octomap>` relationships, then submit the original planning request.
The adapter preserves other effective collision rules, including explicit objects
and attached geometry. Occupancy data is neither fetched nor cleared. Cartesian
planning retains `avoid_collisions=True`; ordinary planning settings and target
tolerances remain unchanged.

The global setting is captured for ordinary plans and rechecked before policy
application, planning, result acceptance, and physical goal dispatch. A setting
change invalidates pending ordinary planning; it does not interrupt execution.
Explicit bypass segments remain independent of global toggle changes.

The scene matrix is shared, so the existing planner serializes reads, policy
updates, and plans. Independent planning clients or collision-matrix writers must
remain idle while this application owns planning. Native sensor updaters may
update occupancy without changing the matrix. Keep uncertain scene updates and
plans blocked until their remote completion is known. An abandoned read-only
request cannot apply a late response. Do not silently fall back after a policy
service failure.

Removed: RTAB-Map snapshot adapter, binary conversion, map-placement/session
validation, map-fetch timeout, conversion worker, mapping-owned collision state,
feature-specific `map_cleanup` setting, and their obsolete tests. Retained: the
independent mapping-height fixes (`RGBD/ForceOdom3DoF=false` and
`Grid/MapFrameProjection=false`) and current command bypass rules.

Offline checks cover default state, UI/service behavior, ownership, matrix rules,
planning sequence, toggle changes, cancellation, and the unchanged motion guard.
Enabling the setting checks occupancy already present in MoveIt; an empty scene
does not provide environmental avoidance.

## 3. Physical lidar frame adapter — offline implementation complete

`sensing/lidar_frame_adapter.py` reuses Humble's vectorized
`tf2_sensor_msgs.do_transform_cloud`. Its standalone launch publishes
`/velodyne/points_sensor`, expressed in the physical `lidar_sensor` frame, using
the original acquisition timestamp and its corresponding TF. It adds no mapping
or arm-executor dependency. The adapter still starts separately; the application
launch now configures its MoveIt consumer in Step 4. Mapping is unchanged.

The launch can publish the previously captured SDK mount calibration through
the standard static-TF component, or reuse an existing verified physical frame
with `publish_mount_tf:=false`. The calibration is in
`config/lidar_mount_calibration.yaml`; verify it against the actual mount before
using it. No hardware or live TF validation was performed for this change.

Processing is bounded by depth-one queues, a 10 Hz attempt limit, a 100,000-point
limit and a 0.5 s age limit checked both before and after conversion. Clouds more
than 50 ms in the future, missing timestamps, unsupported layouts and invalid
transforms are rejected. TF lookup never waits or substitutes the latest pose.
Backward ROS clock jumps and clock-source changes clear dynamic TF history. The
separate adapter process owns no robot commands, planning requests or guard timers.

Offline tests cover real library transforms, moving-body TF, unchanged source
data and acquisition time, malformed/stale input, missing TF and recovery, rate
and size limits, QoS, clock resets and launch/calibration wiring. Physical mount
alignment, actual sensor latency and obstacle clearing remain to be checked.

With hardware available, validate one sensor before combining inputs. For the
rear lidar, verify the declared frame, physical optical/ray origin, point
coordinates, timestamps, and TF chain. Transform both coordinates and frame
consistently; changing only the frame name is insufficient. Reuse calibration
and standard ROS point-cloud transforms. The adapter uses the existing driver
output and needs valid mount calibration and timestamped robot TF; it cannot
infer the physical sensor origin from the point coordinates alone.

Previous live diagnosis found a filtered cloud frame coincident with `odom`,
placing its implied ray origin about 5.8 m from the robot in that capture. Correct
occupied endpoint positions do not validate free-space rays or clearing of moved
obstacles. Recheck this on the active setup before relying on lidar occupancy.
This issue can also affect mapping; it is separate from the corrected height
alignment.

The mapping pipeline also has a fixed arm-exclusion volume approximately
`x=[0.25, 0.70], y=[-0.23, 0.23], z=[-1.00, 1.10]` in `base_link`. It can discard
real nearby objects. Avoid inheriting that broad exclusion into the new arm
obstacle stream; use MoveIt's robot/attached-body filtering where supported.
Keep mapping-specific filters owned by mapping.

## 4. Native MoveIt occupancy — wiring implemented, prerequisites pending

The application launch passes `config/moveit_sensors.yaml` and `use_sim_time`
only to the existing `spot_moveit_config` MoveIt include, using a scoped ROS launch
parameter group. Robot, kinematics, joint limits and planner configuration remain
owned by that package. There is no duplicated MoveIt launch or custom map updater.

Configure `occupancy_map_monitor/PointCloudOctomapUpdater` with corrected
`/velodyne/points_sensor`, 5 cm voxels, 3 m range, 5 Hz maximum update rate, and
3 cm robot-mask padding. The mask padding excludes robot returns rather than
inflating obstacles. Native ray integration and robot/attached-body filtering own
occupancy. `/fault_detector/moveit/filtered_lidar` exposes the updater's filtered
cloud for inspection. Keep the existing lidar adapter separate until physical
mount calibration is verified. No changes to the Spot driver are needed.

`moveit_ros_perception` is now a declared runtime dependency. Its plugin is absent
from the current offline environment and must be installed before sensor use;
the Python package build does not install this system dependency.

**Required model correction:** the current external SRDF has a fixed `body`
root. MoveIt 2.5.9's planning-scene monitor constructs the occupancy monitor in
the robot model frame, even when `octomap_frame: odom` is configured. A standard
floating virtual joint from `odom` to `body` is needed so stored occupancy stays
fixed while Spot moves. Use existing TF for that joint and keep the arm chain,
targets and trajectory execution body-relative. This one-line external SRDF edit
is awaiting approval under the repository-scope rule. Do not use native sensing
against the old body-rooted model.

Launch checks for that floating joint and the perception package before adding
the sensor parameters. A missing prerequisite produces a warning and leaves the
existing arm planner available without this lidar integration. There is no
automatic enablement or runtime retry; install/correct prerequisites before the
next normal application startup.

Offline launch checks expand the actual installed MoveIt include with process
execution blocked. They verify parameter delivery and isolation, identical
robot/planner parameters, and both clock modes. Native MoveIt/FCL checks without
ROS nodes confirm that updated OctoMap geometry respects the existing enabled
and bypass matrices, while self and explicit-object collisions remain active.
An in-memory copy of the actual Spot model with the proposed floating joint also
preserves all six arm variables, limits, gripper configuration and body-relative
forward kinematics across several poses; arm-only state diffs preserve the root.
These checks do not prove runtime TF synchronization. MoveIt's state monitor can
retain an old/default root when TF is unavailable, and Cartesian conversion uses
a separate latest TF lookup. Validate current `odom` to `body` TF, body sway,
planning latency and endpoint accuracy on hardware before relying on this input.

The map updates independently of movement requests; the motion path still reads
only the small collision matrix. The 3 m sensing range does not make this a
rolling map: stored voxels can accumulate as the base travels. No custom pruning,
periodic clearing, synchronous map read, or sensor-readiness wait is added.

Verify frame placement while the base and arm move. Correct sensor-origin
transforms and robot geometry are prerequisites for useful clearing. Do not
promise that unseen or occluded obstacles disappear automatically; measure the
behavior of the installed updater. Add front-left, front-right, and hand inputs
one at a time only after the first source behaves correctly. Check each source's
alignment and robot returns before assessing the combined map.

Sensor-head collision geometry can be added later through the existing robot or
attached-object geometry owner, using calibration/TF and a simple conservative
shape initially. A separate full URDF per head is not the default solution.
Coverage near the omitted head remains a known limitation until geometry exists.

The UI shows the user preference and explicitly says it does not report sensor
freshness. Empty or unobserved space has no environmental coverage. When data
stops, stored occupancy remains; nothing automatically expires or disables the
preference. Visible rays can clear past observations over several updates, but
occluded geometry can persist. Validate coverage through the scene and filtered
cloud during commissioning. This minimal integration adds no second sensor-state
owner or mapping lifecycle and does not continuously replan during execution.

## 5. Validate without changing arm accuracy

1. Run offline policy, command, UI, planner, trajectory, and guard checks.
2. With approval for live diagnostics, inspect the MoveIt scene without executing
   movement. Confirm real obstacle positions, robot filtering, and removal when
   a visible obstacle moves. Measure update and plan-start times.
3. Compare disabled, enabled, and one-shot bypass against the same obstacle and
   target using planning-only tests. Verify self-collision and explicit-object
   rules remain unchanged and that a reachable obstacle can be routed around.
4. With approval for physical tests, use clear-space movements first. Compare
   endpoint accuracy, completion, guard behavior, and latency with the existing
   baseline. Check safe/aligned approaches and final contact-segment policies.
5. Exercise startup without mapping, mapping start/stop, sensor removal, toggle
   changes during preparation, and cancellation. Confirm no accidental movement,
   late policy overwrite, or automatic enablement.

Keep the implementation scoped to this optional planning layer. No planner,
trajectory timing, endpoint tolerance, force threshold, or command architecture
changes are required by the sensor transition.
