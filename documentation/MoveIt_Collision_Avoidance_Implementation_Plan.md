# Map based MoveIt collision avoidance plan

Add optional arm planning against the obstacles already represented by RTAB-Map.
RTAB-Map remains the only environmental map producer. MoveIt receives a snapshot
before planning; existing arm targets, accuracy settings, trajectory checks,
contact handling, and execution guards remain unchanged.

The planning-only feasibility test established map import, bypass, and rejection
of a mapped goal collision. Application integration now follows mapping availability
and the runtime UI control; real command execution validation remains pending.
This is a best-effort aid for known mapped
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
- Show disabled with no map; automatically enable each new mapping session and allow manual on/off through the UI.
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

### Retained implementation and offline checks

The retained production building block is the small
`manipulation/rtabmap_octomap.py` converter. It validates binary occupancy and
constructs a scene diff with complete rigid placement. The optional
`RtabmapCollisionScene` adapter now connects it to the existing planner. Steps 3
and 4 are implemented for active mapping; Step 5 physical validation remains.

The temporary feasibility CLI and its CLI-specific tests were removed after the
live tests. Diagnostic scripts and captures remain under `/tmp`; they are not
application components. Keep the converter unit tests and native binary
compatibility regression: the latter
checks the installed RTAB-Map writer against the installed OctoMap reader with
occupied, free, unknown, and pruned cells at 0.04 m and 0.1 m. The test compiles
into pytest's temporary directory, adds no production C++ package, and starts no
ROS nodes. It explicitly skips if development libraries are absent; a skip is
not proof of compatibility. Direct message/service dependencies retained for
this path are `octomap_msgs` and `std_srvs`; existing mapping also uses `std_srvs`.

Run the offline checks from the repository root with the normal ROS and workspace
environments sourced, without a previous collision experiment's temporary overlay:

```bash
export PYTHONPATH="$PWD:$PYTHONPATH"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  test/test_octomap_binary_compatibility.py test/test_rtabmap_octomap.py \
  test/test_moveit_arm_planner.py test/test_arm_joint_state_source.py \
  test/test_arm_motion_parameters.py test/test_tf_transform_helpers.py
```

Plugin autoload is disabled to avoid an unrelated pytest plugin that opens
sockets in this environment. The retained tests cover binary compatibility,
malformed input rejection, frame/pose validation, and preservation of source
messages and unrelated scene fields. The live evidence below does not replace
production sequencing, cancellation, session, or command-policy tests.

Live tests require an authorized, stationary session with application motion and
RViz planning idle and no competing scene writer or MoveIt sensor updater.
After a timeout or interrupted request, stop the sequence: destroying a local
client does not cancel server work. Confirm completion before another scene
mutation or plan. Clear imported occupancy before returning to ordinary arm use;
the initial feasibility tools had no application bypass.

The feasibility readback checks metadata and placement;
[MoveIt 2.5.9 serializes readback as a full tree](https://github.com/moveit/moveit2/blob/2.5.9/moveit_core/planning_scene/src/planning_scene.cpp#L686),
so its bytes cannot be compared directly with RTAB-Map's binary payload.
Diagnostic telemetry and TF thresholds were test controls, not validated
production policies. Whole-body validity used MoveIt's monitored leg state;
only arm telemetry was independently checked.

### Live feasibility result on 2026 October 7

**Initial result: import and clearing worked, but map placement blocked further validation.**
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

**Follow-up identified by this test:** resolve the mapping pose/height policy in Step 2 and validate it with a
fresh test map before continuing command integration. Evaluate preserving full
odometry height alongside the existing planar navigation constraints, together
with appropriate height filtering. Merely changing `RGBD/ForceOdom3DoF` while
retaining map-frame height projection and the 1.5 m cutoff could discard surfaces
at the robot's current altitude. Do not add an arbitrary vertical offset in the
MoveIt adapter or rewrite existing saved maps to mask the discrepancy.

Diagnostic captures for this session are under `/tmp/spot_map_collision_live`
(`import.log`, `geometry.log`, `clear_check.log`, metadata and serialized snapshots).
At this stage, visual alignment, blocked/clear/reimport planning, and relocation
checks remained unverified. Subsequent height and ready-arm results are recorded
below; application integration was still pending at that stage.

### Height correction retest on 2026 October 7

**At this retest, the height correction passed; arm-path feasibility remained
pending.** After clearing the application's inherited temporary-overlay paths and
relaunching from the workspace, runtime confirmed `RGBD/ForceOdom3DoF=false` and
`Grid/MapFrameProjection=false`, with `Reg/Force3DoF=true`. The test used a fresh
map with Spot sitting and its arm folded. No arm paths or movements were requested.

RTAB-Map's cloud now spans map Z = 3.908–5.605 m, in agreement with filtered lidar
starting near 3.914 m and the robot body at about 4.115 m. The captured binary
snapshot decoded to 5,218 occupied 5 cm voxels. Their body-relative height range
is -0.221 to +1.494 m, with 826 occupied centres within 1.5 m of the body. The
previous roughly 4 m vertical displacement is gone. These measurements establish
coordinate consistency, not independently surveyed accuracy or complete coverage.

The subsequent import fetched a 143,974-byte snapshot in 1.770 s, applied it in
0.142 s, and verified its metadata/placement in a 0.160 s scene readback. Clearing
completed in 0.004 s; readback confirmed empty occupancy and preserved explicit
objects, attachments, and collision rules. The original scene was empty and was
restored to that state.

Arm validity reported `arm_link_el1 / arm_link_sh0` contact both with the imported
map and after clearing. Whole-body validity also reported body/upper-leg contacts
in the seated posture. These are independent of imported occupancy. A later
blocked/clear/reimport comparison needs a normal standing, ready arm pose and a
successful clear-map baseline through the unchanged planner and trajectory checks.

One initial read-only `/move_group/get_parameters` request timed out before any
scene-changing request was submitted. A retry was initially blocked by automatic
approval review. Separate read-only checks established service responsiveness and
empty occupancy; the subsequent approved import and clear both completed. No
scene mutation timed out or remained outstanding.

The filtered cloud still declares a sensor frame coincident with `odom`; its
implied ray origin is about 5.8 m from the robot in this capture. Correct occupied
endpoint heights do not validate free-space ray tracing or removal of moved
obstacles. Keep this existing sensor-origin issue separate from the now-verified
height correction. The previous fixed arm-exclusion coverage limitation also
remains.

Captures are under `/tmp/spot_map_collision_retest_20261007_172215`, including
`geometry.log`, `preflight.log`, `import_verified_health.log`, `clear.log`, and
native decoded occupancy. The subsequent ready-arm comparison is recorded below.
Visual surface alignment, dynamic-obstacle clearing, and localization after an
odometry reset remain separate validation items.

### Ready-arm planning result on 2026 October 7

**Normal planning responds to imported occupancy, and clearing restores the
successful baseline.** This is mechanism evidence, not completion of every
Step 1 gate or validation of executed motion. Spot was standing with its arm
ready and mapping running. No trajectory was executed. All requests completed,
and the imported occupancy was cleared afterward.

The captured map contained 195,266 bytes at 0.05 m resolution, decoding to 6,398
occupied voxels. A 5 cm forward Cartesian plan succeeded with the map present
(fraction 1.0, 16 trajectory points). A longer diagonal Cartesian candidate at
hand position `[1.21, 0.20, 0.32]` m in `body` failed even with the map cleared
(fraction 0.003105); it provides no collision-avoidance evidence. Planner and
trajectory acceptance checks were unchanged.

For normal `GetMotionPlan`, the hand target stayed at `[1.18, 0.20, 0.32]` m in
`body`, using the orientation obtained from the current hand forward kinematics:

| Scene occupancy | Same-target planning result |
| --- | --- |
| Cleared | Success, 102 trajectory points |
| Imported | Failure, MoveIt error 99999 |
| Cleared again | Success, 104 trajectory points |
| Reimported | Failure, MoveIt error 99999 |

Both imported runs reported a valid arm start state. To distinguish the planner's
generic failure from a mapped collision, the saved successful goal state was also
checked directly. Its forward-kinematics hand position was
`[1.18278534, 0.20013897, 0.31969460]` m. With occupancy present, that state was
invalid with `<octomap> / arm_link_fngr` and `<octomap> / arm_link_wr1` contacts;
after the final clear, the same state was valid. This demonstrates rejection of a
mapped collision at the goal; it does not yet demonstrate routing around a wall
to a clear goal or the blocked/bypass Cartesian comparison specified above.

Whole-body checks still reported body/upper-leg self-contacts independently of
the map; the arm group was valid. An initial stale TF was rejected before import.
The temporary test then waited for fresh TF under the same 1.5 s age limit and
passed without relaxing that limit. Checked-plan snapshot fetches took
0.841–1.327 s and application took 0.129–0.181 s. These measured delays must be
considered when integrating preparation into ordinary commands.

Captures are under `/tmp/spot_arm_map_planning_20261007_173657`. Remaining work
includes production sequencing and toggles, the Cartesian blocked/bypass case,
physical endpoint-accuracy regression, visual coverage checks, lidar ray-origin
and moved-obstacle clearing checks, and saved-map placement after odometry resets.
No execution accuracy or continuous obstacle response is established by these
planning-only results.

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

**Implemented for real-system testing:** the mapping runtime automatically enables
checking when a mapping session starts and owns the UI's manual on/off choice.
Without a map or when manually disabled, ordinary planning first clears imported
occupancy. There is no startup switch. The existing runtime supplies an
immutable mapping-session token (process, map, generation, ROS start time). The
planner sequences map fetch → validated scene application → original plan, or
acknowledged occupancy clear → original plan for bypass. Map fetches have a
20 s response timeout; other phases retain the existing 7 s timeout. Binary
snapshot validation/conversion runs in one worker outside the arm execution
lock so force observations and the watchdog continue during preparation. The
worker only builds a scene message; the planner revalidates policy and placement
before applying it. Current TF must postdate the session and be
within -0.1 to 1.5 s of the ROS clock. Body position in map coordinates must stay
within 2 cm and orientation within 0.03 rad of the preparation reference; using
the inverse placement avoids magnifying small body sway at distant map origins.
The executor rechecks placement inside the deferred trajectory-goal builder.

Scene/plan cancellation retains the real service future until completion; an
exceptional or missing response requires re-establishing the application/MoveIt
session. A discarded read-only map fetch or conversion is never imported and
cannot block an explicit bypass after mapping stops. No sensors-parameter polling
or competing occupancy writer is introduced; the existing no-updater MoveIt launch and
exclusive scene ownership remain prerequisites. Saved-map localization stays
disabled for this feature until separately validated.

The October 7 live diagnosis found a healthy 50 Hz force stream, but conversion
of a 509,246-byte map took about 1.35 s under the original shared execution lock.
This delayed force handling beyond its unchanged 0.25 s limit. The same map
fetch took 11.24 s, motivating the separate bounded map-service timeout.
A subsequent read-only observer ran three conversions alongside 533 live force
samples: the longest force gap was 64 ms and there were no stale-force checks
during conversion. No planning scenes or robot commands were sent by that check;
the updated application still needs a real movement retest after relaunch.

Follow-up movement logs showed 14.5–16.5 s of map fetching/conversion versus
0.4–0.5 s of motion planning. The installed RTAB-Map defaults to `map_cleanup:
true`, clearing its assembled OctoMap when no OctoMap topic has subscribers.
Service-only use therefore repeatedly reconstructs the tree. A read-only live
comparison kept the tree present via its existing `/octomap_binary` topic:
subsequent full-map service requests took 0.22 s and 0.16 s. These are fetch-only
measurements, not complete movement-start latency.

The mapping launch now sets `map_cleanup: false`. Requests still update the tree
from current optimized poses, preserving existing session, TF, and cancellation
checks. No application snapshot cache or additional occupancy writer is added.
The initial assembly and graph corrections can still require a full rebuild;
retaining generated map caches increases RTAB-Map memory use. Cropping to an arm
workspace can be considered if transfer/conversion costs become dominant, but
cropping after a full fetch cannot remove the observed reconstruction cost.
The setting requires mapping to be relaunched and movement latency to be retested.

For each checked plan, capture the runtime session/map and a placement reference,
request its current binary
OctoMap, validate the response, resolve current placement, apply the scene diff,
and await completion before calling the existing planning service. Request a map
for every checked planning segment initially; add caching or spatial cropping only
if measured costs justify them. Failed snapshot/TF preparation while checking is
enabled never silently selects bypass. No-map state disables checking before a
new request; loss of mapping during a checked request invalidates that request.

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

When checking is disabled or mapping is unavailable, acknowledge occupancy clearing
before using the original planning request. The global UI setting takes effect
without restart and does not interrupt an executing trajectory. Preparation and
deferred dispatch reject a changed global policy; an explicit per-command bypass
remains independent of global changes. A new mapping session automatically enables
checking, while repeated start requests within a session preserve manual off.
The existing runtime owns the state and publishes it in navigation diagnostics;
a SetBool service changes only that policy, without planning or scene mutation.

**Done when:** import/clear failures, stale sessions, cancellation, and late replies
cannot lead to a plan under the wrong obstacle policy, while existing target and
trajectory behavior is preserved.

## Step 4 Add command and UI control

**Implemented:** the boolean is carried through semantic commands, both ROS
messages, recordings, BT translation, executor planning, corrections, and
checkpoint returns. The one-shot basic-movement checkbox resets before command
admission. Every Move Close to Surface / Wall command bypasses mapped occupancy,
including positive stand-off distances. Final probe-point moves and all final
path waypoints bypass for both manual actions and Execute Probe Point. Saved safe
and aligned approach stages always follow the current global toggle; a blanket
bypass on a complete probe-point command is not propagated into those stages.
The guard's contact retreat selects bypass without changing its contact limits,
stop confirmation, or execution path. Build both packages after the interface
change. See the [real-system test steps](../README.md#optional-mapped-obstacle-checks-for-arm-planning).

Selectively reuse `ignore_environment_collisions=False` and its propagation from
the previous branch. Inspect producers, message definitions, adapters, recording,
BT translation, executor retries/corrections, and consumers together. Rebuild
`fault_detector_msgs` and `fault_detector_spot` together when interfaces change.

| Motion | Map policy when the feature is enabled |
| --- | --- |
| Ordinary basic motion and safe/aligned approach travel | Import and check mapped obstacles |
| Basic motion with its one-shot override | Clear imported occupancy before planning |
| Any Move Close to Surface / Wall or final probe-point path | Always bypass mapped obstacles, including stand-off targets |
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
additional motion-dispatch path. The UI also exposes the runtime
control: Disabled — no map, Enabled, or Disabled. The one-shot override stays
separate from that session-wide setting.

**Done when:** the selected command alone bypasses imported obstacles and the next
ordinary command imports/checks them again while the global control is enabled.

## Step 5 Validate behavior and release the optional feature

**Runtime UI revision:** 204 focused tests pass, including automatic enablement,
manual off/on, no-map ordinary planning, pending-policy invalidation, status
transport, and UI teardown. The isolated application build passes and its
production sources match the checkout. This revision has not been deployed or
validated on the running robot.

**Offline integration result:** both packages built successfully in an isolated
overlay, whose production Python sources match this checkout. The functional
suite passed 2,088 tests; nine failures were reproduced on the unchanged baseline
(reference-capture fixtures and an outdated probe-motion enum expectation). One
additional ROS test failed to open the sandbox's default log directory, then
passed with `ROS_LOG_DIR` under `/tmp`. Repository-wide flake8 and pep257 checks
also fail on the baseline. Collision sequencing, cancellation/late replies,
placement/session checks, transport/UI, contact policy, and deferred dispatch
regressions pass. No integrated live command or robot movement was tested in
this implementation step; physical accuracy and coverage still require the
README's operator tests. No workspace deployment or Git mutation was performed.

Extend relevant existing planner, executor, command-adapter, recording, workflow,
and mapping-lifecycle tests. Add focused tests for binary compatibility, rigid
placement, map-session invalidation, import/clear ordering, late service replies,
and one-shot option consumption. Avoid tests that simply copy implementation.

| Verification | Required result |
| --- | --- |
| Feature disabled | Original requests, target tolerances, trajectory validation, and guards |
| Known blocking obstacle | Checked path fails or routes around it; bypass restores the clear-map baseline |
| Checked then bypass then checked | Import, clear, and reimport take effect in the correct order |
| Map missing or stopped | UI says Disabled — no map; new ordinary requests clear occupancy and plan normally |
| New mapping session | Checking automatically enables; stale in-progress plans are rejected |
| Manual UI toggle | Choice persists within this session; preparation rejects a changed policy |
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
| Runtime policy and UI transport | Existing `RtabmapRuntimeManager`, `std_srvs/SetBool`, navigation diagnostics, and `ui/ros/arm_collision_client.py` |
| Demonstrated map filter corrections | `launch/lidar_rtab_mapping_launch.py` and existing mapping filter owner |
| Command policy | Existing semantic/execution models, ROS adapters, executor, contact workflow factories |
| Transport boolean | `fault_detector_msgs/msg/CommandPayload.msg` and `OperationalIntent.msg` |
| One-shot presentation | `ui/manipulation/controls.py` |
| Feasibility evidence | Recorded results here; retained converter and native compatibility tests under `test/`; temporary diagnostic archived under `/tmp` |

Use the existing `octomap_msgs`, `moveit_msgs`, and `std_srvs` interfaces and declare
direct dependencies where introduced. No driver or `spot_moveit_config` edits are
planned. Carry over the old command/UI behavior and useful tests, not its raw-sensor
freshness monitor, perception plugin build, or collision-matrix machinery unless
the Step 1 evidence establishes a specific need.
