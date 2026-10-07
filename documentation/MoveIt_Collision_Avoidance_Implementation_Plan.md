# Map based MoveIt collision avoidance plan

Add optional arm planning against the obstacles already represented by RTAB-Map.
RTAB-Map remains the only environmental map producer. MoveIt receives a snapshot
before planning; existing arm targets, accuracy settings, trajectory checks,
contact handling, and execution guards remain unchanged.

The first milestone is a planning-only feasibility test. Continue with application
integration only if the imported map provides useful arm coverage without false
robot collisions or excessive delay. This is a best-effort aid for known mapped
obstacles, not continuous obstacle detection or a replacement for the collision guard.

## Baseline and scope

Both repositories use `feature/map_based_arm_collision`. The inspected baselines
are `fault_detector_spot` at `5015dd5` and `fault_detector_msgs` at `8d2eef2`.
The prior `feature/move_it_collision` branch is a reference for reusable command
and UI work, not the implementation to restore wholesale.

- First support an active RTAB-Map mapping session. Enable saved-map localization
  through the same path once its placement and availability checks pass.
- Lidar attachment alone is insufficient: RTAB-Map and valid robot placement in
  the selected map must be available for checked planning.
- Keep normal planning on `GetMotionPlan` and straight approaches on
  `GetCartesianPath`, with `avoid_collisions=True`.
- Preserve the executor option and one-shot basic-movement UI override requested
  previously. Explicit contact motions can bypass mapped obstacles.
- Default the feature to disabled until validation is complete.
- Add no raw sensor fusion, MoveIt perception plugins, second map generator,
  background map refresh, or new planning action server.
- Make no Spot driver changes. Sensor-head collision geometry remains a later task.

Implementation begins with Step 1. Node starts, live planning-scene changes, and
physical tests require authorization for the relevant test session. A diagnostic
that plans without executing still modifies the shared MoveIt scene.

## Design and ownership

The perception path is:

`Existing lidar and mapping filters → RTAB-Map binary OctoMap → snapshot adapter → existing MoveIt planning scene`

The movement path remains:

`UI or workflow → CommandController and ROS command transport → Behavior Tree → ArmMovementExecutor → MoveItArmPlanner → existing execution and guards`

`HelperInitializer` already owns both `RtabmapRuntimeManager` and
`RobotCommandResources`. Inject a narrow read-only view of the existing map
runtime into the planner's snapshot adapter. Reuse its active map, running mode,
and process/session identity. Do not create another map lifecycle manager or ROS
status interface. A selected map name alone does not mean mapping is running;
Nav2 availability is not an arm-planning prerequisite.

One small adapter owns RTAB-Map service/message conversion. `MoveItArmPlanner`
owns the asynchronous prepare-and-plan sequence. The executor retains ownership
of movement, cancellation, completion, and runtime safety. Use existing TF and
geometry helpers for placement; do not duplicate calibration or pose mathematics.

## Map import and bypass

Use RTAB-Map's `octomap_msgs/srv/GetOctomap` binary export, expected at
`/rtabmap/octomap_binary` with this launch. Confirm the actual service name and
availability during the feasibility test. The 2D navigation grid is not suitable.

The installed RTAB-Map implementation exports a tree identified as `ColorOcTree`;
MoveIt 2.5.9 accepts `OcTree`. Verify binary occupancy compatibility with the actual
libraries before allowing a binary-only type normalization. Preserve occupied
and free cells, resolution, and placement. Reject unsupported or malformed input;
never relabel a full colored-tree serialization as an occupancy tree.
Reject empty startup responses, nonpositive/nonfinite resolution, and unexpected
map frames before importing. Missing occupancy must not become an apparently
checked plan with no obstacle map.
MoveIt supports importing and replacing an OctoMap directly through its
[planning-scene implementation](https://github.com/moveit/moveit2/blob/2.5.9/moveit_core/planning_scene/src/planning_scene.cpp#L1152).

For checked planning, wrap the map in `OctomapWithPose`: set the wrapper frame to
`body` and its origin to the current `body ← map` transform. Keep the voxel data
in map coordinates. Use the full rigid transform, not a planar approximation or
the transform from when old cells were first observed. Apply a planning-scene
diff with both scene and robot-state diff flags set, preserving other objects,
attached bodies, robot state, and collision rules. Wait for acknowledgment before
submitting the original planning request.

For bypass, prefer an acknowledged `/clear_octomap` request followed by ordinary
planning. MoveIt 2.5.9 removes the OctoMap world object without removing explicit
collision objects or changing the Allowed Collision Matrix; verify that behavior
against an imported map in Step 1. This replaces the previous matrix-editing
approach and requires exclusive application ownership of MoveIt's occupancy.
The next checked request must reimport its map. See the
[clear implementation](https://github.com/moveit/moveit2/blob/2.5.9/moveit_ros/planning/planning_scene_monitor/src/planning_scene_monitor.cpp#L606).

Do not start MoveIt camera/lidar updaters or another map publisher alongside this
path: they could repopulate a cleared scene or overwrite the imported snapshot.
If clearing fails or has an uncertain outcome, do not submit a supposedly
bypassed plan. Never implement bypass by setting Cartesian `avoid_collisions=False`.

## Step 1 Prove map import without executing movement

Deliver a small diagnostic and isolated compatibility checks before changing
production command interfaces or copying the previous implementation.

1. Validate binary conversion offline with the installed OctoMap libraries. Use
   known occupied/free cells and nontrivial translation and rotation. Check that
   unsupported formats fail clearly. A temporary C++ test in `/tmp` is acceptable;
   it does not imply adding a production C++ package.
2. In an authorized test session, obtain the actual RTAB-Map snapshot. Record
   frame, encoding, resolution, size, and request/import duration. Confirm current
   `body ← map` placement and the active map/session.
3. Import into a clean MoveIt session with sensor updaters disabled and application
   motion/RViz planning idle. Read back the OctoMap and check alignment in RViz.
   Place visible references beside the robot and near the arm, not only far walls.
4. Check that the measured robot start state is valid with the map present.
   Examine ground near feet, robot returns, and the mapping arm-exclusion volume.
5. Choose an unchanged hand target in `body`, including its correct orientation,
   that succeeds without map occupancy. Choose a target whose straight path crosses
   a mapped obstacle. Compare imported-map Cartesian planning, acknowledged clear
   and planning, then reimport and planning: expect blocked, success, blocked.
   Ordinary planning may instead route around the obstacle. Execute none of these paths.
6. Verify that clearing preserves robot collision rules and an independently
   inserted test collision object. Confirm fresh import places obstacles correctly
   after an approved base repositioning or localization change, if available.

**Proceed when:** binary import works; known surfaces align; useful nearby
obstacles survive the mapping filters; the robot start state is valid; the
blocked/bypass comparison passes; and measured import latency is acceptable for
ordinary arm use. Record timings rather than promising a performance gain.

**Reassess when:** success requires blanket collision exceptions, a larger blind
region around the arm, or substantial mapping reconstruction. Keep the previous
branch available as the alternative. Do not build both approaches into the new
branch merely to postpone this decision.

### Step 1 implementation and testing

The first implementation adds `scripts/check_rtabmap_moveit.py` and the small
`manipulation/rtabmap_octomap.py` converter. Normal application commands do not
use them yet. The diagnostic performs exactly one import or clear, reads the scene
back, reports whole-robot and arm collision validity, and optionally asks the
existing `MoveItArmPlanner` for one plan. It has no trajectory execution client.
Direct dependencies are `octomap_msgs`, `std_srvs`, and `rcl_interfaces`.

The installed RTAB-Map 0.23.8 binary writer was tested against the installed
OctoMap reader using occupied, free, unknown, and pruned cells at 0.04 m and 0.1 m.
The converter validates binary record structure before native decoding and keeps
the complete rigid placement. This proves binary compatibility, not live map
alignment, coverage, or planning usefulness. Those remain the feasibility gate.

Run the offline checks from the repository root in a terminal with the ROS and
workspace environments sourced:

```bash
source /opt/ros/humble/setup.bash
source /home/marcel/Projects/spot/spot_ws/install/setup.bash
export PYTHONPATH="$PWD:$PYTHONPATH"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  test/test_octomap_binary_compatibility.py \
  test/test_rtabmap_octomap.py test/test_rtabmap_moveit_diagnostic.py \
  test/test_moveit_arm_planner.py test/test_arm_joint_state_source.py \
  test/test_arm_motion_parameters.py test/test_tf_transform_helpers.py
python3 scripts/check_rtabmap_moveit.py --help
```

These checks start no ROS nodes. The C++ compatibility check compiles into a test
temporary directory and explicitly skips if its development libraries are absent;
a skip is not proof of compatibility. Plugin autoload is disabled to avoid an
unrelated pytest plugin that opens sockets in this environment.

For an **explicitly approved live diagnostic session**, use the normal driver,
mapping, and MoveIt setup. Keep the base and arm stationary, preserve the active
map session, and leave application motion and RViz planning idle. No other map
publisher or scene writer may run. The diagnostic checks the MoveIt `sensors`
parameter, but that alone cannot detect external scene writers. Do not source the
previous collision branch's temporary overlay. A separate local build for this
step is available under `/tmp/spot_map_collision_step1/install`.

1. Source that overlay after the workspace and keep the checkout first on
   `PYTHONPATH` as above. Confirm the map service using
   `ros2 service list -t`; override `--map-service` if its name differs.
2. Run `python3 scripts/check_rtabmap_moveit.py import`. Record printed snapshot
   metadata, service timings, and collision contacts. In RViz's MoveIt planning
   scene, inspect occupancy alignment around the arm, nearby objects, ground,
   and feet. A current snapshot timestamp does not establish fresh observations.
3. Add an independent box through RViz's planning-scene object controls, away
   from the current robot. Apply it, then leave RViz scene editing idle. Run
   `python3 scripts/check_rtabmap_moveit.py clear`. Confirm the box ID is printed
   and still visible, while map occupancy disappears. The diagnostic compares
   explicit object geometry, attachments, and collision rules before/after.
4. Select a previously reachable **hand** pose in `body`, in metres, with its
   existing quaternion orientation. Keep that identical target and measured
   start for all three commands below; replace the capitalized placeholders
   with numbers. Choose a clear endpoint whose straight approach crosses a
   mapped surface. Do not choose a probe-tip pose or guess a new orientation.

   ```bash
   python3 scripts/check_rtabmap_moveit.py import --cartesian --target X Y Z QX QY QZ QW
   python3 scripts/check_rtabmap_moveit.py clear --cartesian --target X Y Z QX QY QZ QW
   python3 scripts/check_rtabmap_moveit.py import --cartesian --target X Y Z QX QY QZ QW
   ```

   Run each separately and inspect its result before continuing. Expect blocked,
   success, blocked. A failure alone may be IK or another planning restriction;
   the clear-map success is necessary evidence. Omit `--cartesian` to try ordinary
   planning around the obstacle. No returned trajectory is executed.
5. After completed requests, run `python3 scripts/check_rtabmap_moveit.py clear`
   before returning to ordinary application arm use. Remove the diagnostic box
   through RViz after verifying preservation. Map occupancy persists in the shared
   MoveIt scene when the script exits; this branch has no application bypass yet.

Exit code 0 means the requested checks/plan passed, 2 means an invalid arm start
or an unsuccessful optional plan, and 1 means the diagnostic stopped on an error.
The script requires fresh, complete arm position/velocity/effort telemetry and
compares arm positions with MoveIt's state within 0.02 rad. Its TF age limit is
1.5 s; translation/rotation changes above 0.02 m/0.03 rad reject the result. These
are diagnostic thresholds, not validated production policies. Full-body validity
is reported separately so ground/feet contacts remain visible; the arm-group
result gates the optional arm plan. Only arm telemetry is independently checked;
the whole-body readout uses MoveIt's monitored leg state. Validate all unexplained
contacts before passing the feasibility gate.

After a timeout or interrupted request, **stop the sequence**. A local client
shutdown does not cancel server work. Confirm completion or re-establish the
MoveIt session before another import, clear, or plan. The diagnostic does not
retry or automatically clear after an uncertain result. Its readback checks
metadata and placement; [MoveIt 2.5.9 serializes readback as a full tree](https://github.com/moveit/moveit2/blob/2.5.9/moveit_core/planning_scene/src/planning_scene.cpp#L686),
so comparing the returned bytes with RTAB-Map's binary payload would be incorrect.

### Live feasibility result on 2026 October 7

**Status: import and clearing work, but the feasibility gate has not passed.**
The authorized session used the running Spot driver, MoveIt, and RTAB-Map mapping.
No physical movement was commanded. The original scene had no map, explicit
objects, or attachments; it was restored to that state after the checks, with
the original collision rules preserved. No mapping settings were changed.

| Check | Observed result |
| --- | --- |
| RTAB-Map export | Binary ColorOcTree in `map`, 0.05 m resolution, 211,550 bytes |
| Native occupancy decode | 9,448 occupied leaves; occupied/free encoding accepted |
| Snapshot fetch | 1.563–2.149 s in two requests |
| MoveIt import | Accepted in 0.265 s; readback metadata and full placement matched |
| Readback | 0.258 s after import |
| Arm collision validity | Valid both with occupancy and after clearing |
| Whole-body collision validity | Body/upper-leg contacts both before and after clearing; not an imported-map effect |
| Clear occupancy | Acknowledged in 0.011 s; temporary independent box, attachments, and ACM preserved |
| Cleanup | Temporary box removed; readback confirmed empty occupancy and original scene contents/rules |
| Clear-map Cartesian baseline | A 5 cm forward hand target with unchanged orientation returned fraction 0.0 |

The decisive problem is the existing map's vertical placement. Native occupied
voxel centres span map Z = -0.625 to +1.475 m, while TF puts the robot body at about
Z = 4.072 m and the hand at 4.327 m. The closest occupied centre is 3.84 m from the
hand. RTAB-Map's own `/cloud_map` has the same low height range, whereas the current
filtered lidar points begin near Z = 3.45 m in a frame coincident with `odom`.
Thus the mismatch exists before MoveIt import. A valid arm collision result in
this scene does not establish useful avoidance.

Runtime confirmed `Reg/Force3DoF=true` and `RGBD/ForceOdom3DoF=true`, with
`map <- odom` translation Z = 0 and `odom <- body` Z approximately 4.07 m.
The installed RTAB-Map source uses the former parameter for planar optimization,
and the latter projects incoming odometry to XY/yaw before storing poses
(`rtabmap/corelib/src/Rtabmap.cpp`, `Optimizer.cpp`, and `Transform.cpp`). This
explains the lost height in mapped geometry while the external TF chain retains
the robot's altitude. The OctoMap itself is still 3D.

The arm was near its stowed joint configuration during the baseline plan. Its
measured `arm_sh1` was -3.12046 rad, below the existing -3.10669 rad trajectory
safety floor. The Cartesian service did not identify its rejection cause; do not
attribute fraction 0.0 exclusively to this limit or relax the guard to get a pass.
A later comparison needs a normal ready arm pose established through the existing
movement controls, followed by a successful clear-map baseline.

**Next:** resolve the mapping pose/height policy in Step 2 and validate it with a
fresh test map before continuing command integration. Evaluate preserving full
odometry height alongside the existing planar navigation constraints, together
with appropriate height filtering. Merely changing `RGBD/ForceOdom3DoF` while
retaining map-frame height projection and the 1.5 m cutoff could discard surfaces
at the robot's current altitude. Do not add an arbitrary vertical offset in the
MoveIt adapter or rewrite existing saved maps to mask the discrepancy.

Diagnostic captures for this session are under `/tmp/spot_map_collision_live`
(`import.log`, `geometry.log`, `clear_check.log`, metadata and serialized snapshots).
Visual alignment, blocked/clear/reimport planning, and relocation checks remain
unverified. The evidence currently supports the import mechanism, not enabling
the feature for ordinary arm commands.

## Step 2 Resolve only mapping issues demonstrated by the test

The current mapping configuration already enables 3D occupancy and ray tracing.
Its `Grid/MaxObstacleHeight=1.50` is a height cutoff, not a sensing radius. If
needed, raise it to 3.0 and regenerate the affected occupancy data; changing the
parameter does not guarantee that saved local grids are rebuilt. Check whether
including overhead objects also changes the 2D navigation projection.

The fixed arm exclusion removes every return inside approximately
`x=[0.25, 0.70], y=[-0.23, 0.23], z=[-1.00, 1.10]` in `base_link`.
Real objects inside it can be absent unless observed from another robot pose.
Imported occupancy does not receive MoveIt's live-sensor self-filter. Full
RTAB-Map occupancy can also contain ground; do not assume an obstacles-only map.

If ray origins prove relevant to map quality, reuse the SDK-derived physical
lidar calibration and standard ROS cloud conversion investigated on the previous
branch. Apply it upstream of mapping as a separate, validated change. Keep the
existing mapping self-filter responsibility and verify point alignment and clearing.
Do not combine camera additions, sensor-head geometry, general SLAM tuning, and
map import into this milestone.

**Done when:** the specific coverage or representation problem has an evidenced
small correction, or is explicitly accepted as a limitation of this optional aid.

## Step 3 Integrate snapshot preparation into the existing planner

For each checked plan, capture the runtime session/map and a placement reference,
request its current binary
OctoMap, validate the response, resolve current placement, apply the scene diff,
and await completion before calling the existing planning service. Request a map
for every checked planning segment initially; add caching or spatial cropping only
if measured costs justify them. Missing maps must never silently select bypass.

Historical cell age is acceptable for this feature. RTAB-Map's snapshot service
stamps its response at request time even when cells are old; this is not evidence
of recent sensing. Require a working current map session and valid localization/TF,
not a new camera or lidar acquisition after every request. Check readiness in the
existing runtime and actual transform observations, not merely a cached map name.
After switching or restarting maps, require placement established in the new
session. A cached transform under the same `map` frame name can belong to the
previous map and is insufficient.

Because MoveIt plans in `body`, imported map placement is a snapshot. Require the
base to remain stationary during preparation and planning. Recheck session and
meaningful placement changes before accepting a result and dispatching execution.
Map switches, restarts, localization jumps during the snapshot request or planning,
and unavailable placement invalidate
the prepared plan. Reuse existing measured-state/TF observations; set small
documented age and displacement tolerances from the feasibility measurements so
normal body sway does not cause unnecessary rejection. Continuous map tracking or
replanning during execution is outside this feature.

Keep a single outstanding prepare/plan sequence. A local future cancellation does
not cancel a remote ROS service. After timeout or cancellation, discard obsolete
results and do not start another scene mutation or plan until outstanding server
work has completed or the planning session has been explicitly re-established.
This applies to map application, clearing, and planning. Use bounded timeouts and
existing failure feedback; add no automatic retry loop or second execution owner.

When the feature is disabled for the session, retain the original planning path
and use a clean MoveIt instance without imported occupancy. Session configuration
changes require a matching application/MoveIt restart. Per-command bypass remains
available without restarting and requires no RTAB-Map service or localization.

**Done when:** import/clear failures, stale sessions, cancellation, and late replies
cannot lead to a plan under the wrong obstacle policy, while existing target and
trajectory behavior is preserved.

## Step 4 Add command and UI control

Selectively reuse `ignore_environment_collisions=False` and its propagation from
the previous branch. Inspect producers, message definitions, adapters, recording,
BT translation, executor retries/corrections, and consumers together. Rebuild
`fault_detector_msgs` and `fault_detector_spot` together when interfaces change.

| Motion | Map policy when the feature is enabled |
| --- | --- |
| Ordinary basic motion and safe approach travel | Import and check mapped obstacles |
| Basic motion with its one-shot override | Clear imported occupancy before planning |
| Explicit final contact or custom probe segment | Bypass only where the motion definition requests contact |
| Guarded contact backoff | Preserve the deliberate contact-recovery policy |
| SDK motions outside MoveIt | No new map checking implied |

Do not mark an entire multi-stage workflow unchecked because its final segment
requires contact. Preserve each segment's choice through retries and recovery.
Bypass never changes force/contact guards, joint limits, speed, goal tolerances,
self-collision checking, or independently modeled objects and attachments.

The basic-movement checkbox captures the option on submission and immediately
clears, including when the request is rejected. Unrelated base, gripper, and saved
workflow actions do not consume it. Reuse ordinary command feedback for map
unavailability. Add no confirmation flow, admission-tracking state machine, or
new live global-settings UI.

**Done when:** the selected command alone bypasses imported obstacles and the next
ordinary command imports/checks them again.

## Step 5 Validate behavior and release the optional feature

Extend relevant existing planner, executor, command-adapter, recording, workflow,
and mapping-lifecycle tests. Add focused tests for binary compatibility, rigid
placement, map-session invalidation, import/clear ordering, late service replies,
and one-shot option consumption. Avoid tests that simply copy implementation.

| Verification | Required result |
| --- | --- |
| Feature disabled | Original requests, target tolerances, trajectory validation, and guards |
| Known blocking obstacle | Checked path fails or routes around it; bypass restores the clear-map baseline |
| Checked then bypass then checked | Import, clear, and reimport take effect in the correct order |
| Map missing, stopped, switched, or restarted | Checked request fails clearly; explicit bypass remains available |
| Base moved or map placement changed | Previous prepared trajectory is rejected; next request uses current placement |
| Independent world object and self-collision | Still checked during map bypass |
| Cancel, timeout, late map/clear/plan response | No movement and no overlap with a new policy-changing request |
| Intentional contact and backoff | Existing guarded execution and completion semantics |

Build in an isolated overlay first. Do not use the previous branch's temporary
installed overlay as evidence for this branch; it contains different interfaces
and perception code. Approved live validation starts with planning-only tests,
then familiar small movements in clear space. Compare endpoint error and success
with the feature disabled and enabled; do not relax existing acceptance thresholds.
Check saved-map localization before advertising that mode as supported.

**Done when:** useful obstacle avoidance, reliable bypass, unchanged endpoint
acceptance, and acceptable added delay are demonstrated on the intended robot.
Newly moved or unmapped objects remain subject to mapping coverage and the existing
execution guard; this feature does not promise immediate dynamic-obstacle response.

## Expected change locations

| Responsibility | Existing location or narrow addition |
| --- | --- |
| Snapshot preparation and service sequencing | `manipulation/moveit_arm_planner.py` and one small map snapshot adapter |
| Authoritative map/session view | `mapping/runtime/rtabmap_runtime_manager.py` |
| Dependency wiring and shared TF | `application/behaviour_tree/behaviours/helper_initializer.py` and `robot_command_resources.py` |
| Settings and launch opt-in | `config/arm_motion.yaml` and `launch/fault_detector_launch.py` |
| Demonstrated map filter corrections | `launch/lidar_rtab_mapping_launch.py` and existing mapping filter owner |
| Command policy | Existing semantic/execution models, ROS adapters, executor, contact workflow factories |
| Transport boolean | `fault_detector_msgs/msg/CommandPayload.msg` and `OperationalIntent.msg` |
| One-shot presentation | `ui/manipulation/controls.py` |
| Feasibility evidence | One planning-only diagnostic under `scripts/` and focused tests under `test/` |

Use the existing `octomap_msgs`, `moveit_msgs`, and `std_srvs` interfaces and declare
direct dependencies where introduced. No driver or `spot_moveit_config` edits are
planned. Carry over the old command/UI behavior and useful tests, not its raw-sensor
freshness monitor, perception plugin build, or collision-matrix machinery unless
the Step 1 evidence establishes a specific need.
