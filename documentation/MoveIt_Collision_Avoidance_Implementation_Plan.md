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
or arm-executor dependency to cloud conversion. One `LidarAdapterRuntime`, owned
by `HelperInitializer`, now starts/stops the launch asynchronously based on the
existing collision preference and mapping/localization process owners. Both
application-managed consumers use this corrected source; mapping keeps its
existing downstream arm-exclusion filter. Standalone mapping retains its raw
default unless `raw_lidar_topic` is explicitly overridden.

Neither consumer active means the adapter stops; either or both active mean one
adapter. Nav2 and pending mapping operations retain the source. A steady-time
timer schedules reconciliation on the existing runtime worker, so process waits
never block ROS callbacks or BT ticks. Existing publishers are reused without
waiting for fresh scans. The default adapter exposes a standard Trigger shutdown
service for cooperative release of a manually started instance. Older processes
need one manual stop after rebuilding. Owned process groups are cleaned up on
application shutdown; manually started instances should be released by disabling
both consumers before exit, while service transport is still available.
The standalone launch shuts down its mount component when the adapter exits.
DDS discovery cannot guarantee uniqueness during simultaneous manual launches;
duplicate publishers are reported, not supplemented with another adapter.

The launch can publish the previously captured SDK mount calibration through
the standard static-TF component, or reuse an existing verified physical frame
with `publish_mount_tf:=false`. The calibration is in
`config/lidar_mount_calibration.yaml`; verify it against the actual mount before
using it. No hardware or live TF validation was performed for this change.

Processing is bounded by depth-one queues, a 5 Hz attempt limit, a 100,000-point
limit and a 0.75 s age limit checked both before and after conversion. The age
limit accommodates user-measured live lidar delays around 0.4 s with spikes to
0.56 s, without waiting or changing acquisition timestamps. Clouds more
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

## 4. Native MoveIt occupancy — offline verification complete

The application launch passes `config/moveit_sensors.yaml` and `use_sim_time`
only to the existing `spot_moveit_config` MoveIt include, using a scoped ROS launch
parameter group. Robot, kinematics, joint limits and planner configuration remain
owned by that package. There is no duplicated MoveIt launch or custom map updater.

Configure `occupancy_map_monitor/PointCloudOctomapUpdater` with corrected
`/velodyne/points_sensor`, 5 cm voxels, 3 m range and 5 cm robot-mask padding.
The adapter's monotonic rate limit caps input at 5 Hz. Disable the native ROS-time
throttle (`max_update_rate: 0.0`) to prevent stalled updates after backward clock
jumps during recording replay. The mask padding excludes robot returns rather than
inflating obstacles. Native ray integration and robot/attached-body filtering own
occupancy. `/fault_detector/moveit/filtered_lidar` exposes the updater's filtered
cloud for inspection. Verify mount calibration before enabling either consumer.
No changes to the Spot driver are needed.

A live capture on 2026-10-08 reproduced wrist collisions despite correct point
masking: with the previous 3 cm padding, retained points generated 5 cm cells
intersecting wrist geometry. Offline checks with the installed ShapeMask,
OctoMap and FCL libraries, all 31 robot collision shapes and acquisition-time
TF removed those intersections at 5 cm padding, excluding 60 additional points
from the same capture. This is a bounded correction, not proof for every pose.
It also excludes nearby external returns within the padded robot shapes.
After loading the new configuration, reconstruct the scene from fresh data;
old occupied cells may persist until cleared. Custom sensor-head geometry is
still absent and cannot be supplied by publishing a probe TF alone.

`moveit_ros_perception` is a declared runtime dependency. The configured
lidar/OMPL path now uses matching MoveIt 2.5.10 libraries and `moveit_msgs` 2.2.3.
The native, node-free loader check parses the actual YAML, discovers and loads
the point-cloud plugin, constructs it, and destroys it successfully. The earlier
2.5.10/2.5.9 library mismatch is resolved. The Python package build does not
install or repair system dependencies; package discovery alone is not a load test.

**Model correction implemented:** `spot_moveit_config/config/spot.srdf` now
declares the floating `odom_joint` from `odom` to `body`. MoveIt 2.5.9's
planning-scene monitor constructs the occupancy monitor in
the robot model frame, even when `octomap_frame: odom` is configured. A standard
floating joint keeps stored occupancy fixed while Spot moves. Existing TF supplies
the joint pose; the arm chain, targets and trajectory execution stay body-relative.
This is the only change made to the external configuration for this step; existing
collision exemptions and joint-limit edits were preserved. Rebuild that package
before deployment. Do not use native sensing against the old body-rooted model.

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
The actual updated Spot model was compared with its pre-change SRDF using native
MoveIt libraries without ROS initialization. It preserves all six arm variables,
limits, gripper configuration and body-relative forward kinematics across several
poses; arm-only state diffs preserve the root. The scene frame is now `odom`.
The normal workspace installation also passes launch expansion with process
execution blocked: the corrected topic, sensor parameters and floating-joint
model reach `move_group`, with native trajectory execution still disabled.
All 220 targeted offline tests pass. Native YAML parsing verifies the actual
sensor parameter names, values and types. The collision-policy and Spot-model
checks also pass after rebuilding against MoveIt 2.5.10 and regenerating their
serialized messages with `moveit_msgs` 2.2.3. No ROS nodes were started, so plugin
initialization with TF, cloud ingestion and runtime timing remain unverified.
These checks do not prove runtime TF synchronization. MoveIt's state monitor can
retain an old/default root when TF is unavailable, and Cartesian conversion uses
a separate latest TF lookup. Validate current `odom` to `body` TF, body sway,
planning latency and endpoint accuracy on hardware before relying on this input.

The consumed ROS interfaces were compared before and after the package update.
`GetCartesianPath` adds velocity/acceleration scaling fields; their generated
zero defaults mean 1.0 internally, preserving the existing request behavior.
The executor still stretches trajectory timing to the requested duration. Use a
fresh application and MoveIt process for the next authorized session so both
sides use the new service definition.

A separate dependency scan found that the unused CHOMP planner still requires
`libchomp_motion_planner.so.2.5.10` while its package is 2.5.9. The configured
OMPL pipeline and point-cloud plugin resolve successfully and do not use CHOMP.
That unrelated installation issue was left unchanged.

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

The lidar-only implementation and offline checks are complete. Two hardware
validation stages remain: passive sensing first, then controlled arm/base tests
covering avoidance, bypasses, accuracy and response time. Fixes or calibration
adjustments depend on those observations; there is no reliable completion-time
estimate before seeing the real data.

The broader requested coverage is not complete: front-left, front-right and hand
cameras still need adding and validating individually after the lidar passes.
Sensor-head collision geometry remains a later extension. The unused CHOMP
dependency issue does not block the current lidar/OMPL path.

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

### First hardware session: observe before commanding movement

These steps are for a later authorized hardware session, after verification of
`config/lidar_mount_calibration.yaml` against the physical
mount. They have not been executed during offline development.
Rebuild `fault_detector_spot` after taking the new RViz preset so the installed
configuration is available to the command below.

1. Use the normal driver and application with Spot stationary. Enable the UI
   collision setting without submitting movement; this starts the adapter
   automatically. Mapping is unnecessary. After updating, stop any older manual
   adapter once so the application can own its replacement. Do not launch a
   second adapter or static-TF publisher.
2. Open the passive collision view from this package, without launching another
   MoveIt server or synthetic joint-state publishers:

   ```bash
   rviz2 -d "$(ros2 pkg prefix fault_detector_spot)/share/fault_detector_spot/config/arm_collision.rviz"
   ```

   The preset uses **Fixed Frame: odom**, the planning scene on
   `/monitored_planning_scene`, the robot's collision geometry, and corrected
   lidar points on `/velodyne/points_sensor`. Enable **Filtered lidar** to inspect
   `/fault_detector/moveit/filtered_lidar`; both clouds use **Best Effort**.
   Disable cloud displays when inspecting occupied voxels alone. TF shows `body`
   and `lidar_sensor`; select the actual sensor frame if using a custom name.
   The display has no planning, execution or navigation controls. It reads the
   initial scene through `get_planning_scene` and subscribes to subsequent
   updates; it does not modify MoveIt's scene.
3. Verify that the physical lidar origin is at the rear mount and visible
   surfaces align with the scene voxels. Check that the body and arm do not
   leave occupied trails. Move only an external panel within 3 m into a clear
   line of sight: its new location should appear and rays reaching beyond its
   old location should clear old observations over subsequent updates. Occluded
   space is not expected to clear. Record the observed delay rather than assuming
   that a 5 Hz input means 200 ms clearing.
4. Toggle the UI setting off without submitting movement. With mapping and
   localization also stopped, expect the adapter and mount container to exit;
   stored occupancy remains. Re-enable and expect one adapter to return. With
   mapping/localization running, toggling collision checking off must leave the
   same adapter running. With collision checking on, stopping mapping must also
   retain it. Stopping the last consumer must stop it. Check corrected-topic
   publisher count as well as RViz; the toggle reports intent, not sensor readiness.
   The requested collision policy is applied when the application prepares a
   plan. There is no application plan-only action.

Stop this validation stage at passive observation. Base motion, arm avoidance,
endpoint accuracy, latency and contact-bypass behavior belong to the subsequent
authorized motion tests. Keep cameras out of the initial test so the rear lidar
can be evaluated on its own.
