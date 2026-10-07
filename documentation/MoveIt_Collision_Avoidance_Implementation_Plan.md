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
