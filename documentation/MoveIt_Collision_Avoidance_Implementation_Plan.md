# MoveIt collision avoidance implementation plan

Add sensor-based environmental collision checking to the existing Spot arm planning path, while preserving an explicit per-motion option to use the current behavior. Keep self-collision checking, motion constraints, trajectory validation, contact guards, and physical execution under their existing owners.

The recommended first release uses the existing MoveIt planner and one live lidar input. It supports stationary-base arm operations, deliberate contact motions, and a one-shot override for basic UI movements. Front-left, front-right, and hand depth inputs follow after the single-source behavior is verified. Physical sensor-head geometry is an independent later increment; calibrated sensor poses are required as soon as measurements are used.

Status: proposed development plan, reviewed against the current checkout and locally installed ROS 2 Humble interfaces on 7 October 2026. This document does not authorize deployment, node starts, live validation, or Git operations.

## Requirements and boundaries

The implementation must satisfy these contracts:

1. Normal arm travel can plan around observed environmental obstacles or fail when no acceptable path exists.
2. A designated motion can ignore the newly introduced sensor environment and retain the existing planning and execution checks. This is not a request to disable MoveIt planning.
3. A basic-movement UI override affects one submitted command only. Other commands and later requests retain their own policies.
4. Close-to-surface and custom final probe motions can retain intentional-contact behavior. No target segmentation is required for this first version.
5. Mapping need not be active. Availability depends on usable perception, transforms, robot state, and the validated sensor configuration.
6. Sensor geometry can be absent during initial development and added later through the existing sensor model and attachment lifecycle. No separate complete robot URDF is needed per head.
7. Commands continue through the application command infrastructure. Perception or planning components never send physical Spot commands.

The first release does not provide continuous obstacle avoidance during execution, whole-body planning, environmental protection for every Spot SDK command, or proof that unseen space is free. A collision-free plan is relative to its geometry, observations, tolerances, and checking resolution.

“Current behavior” means the existing validation rules and execution guards, not a bit-for-bit identical trajectory from a sampling-based planner. Later additions to robot or attachment geometry are separately reviewed changes to that baseline.

## Current implementation and consequences

Paths below are relative to the package root unless prefixed with another package name.

| Existing component | Observed behavior | Consequence |
| --- | --- | --- |
| `manipulation/moveit_arm_planner.py` under `fault_detector_spot` | Uses `GetMotionPlan` and `GetCartesianPath`; Cartesian requests set `avoid_collisions=True` | Preserve both planning modes; the Cartesian boolean is not the desired environmental-only policy |
| `manipulation/arm_movement_executor.py` | Receives planned trajectories and sends movement through the existing execution lifecycle | MoveIt remains a planning provider, not a new physical execution owner |
| `spot_moveit_config/launch/move_group.launch.py` | Disables MoveIt trajectory execution and has no sensor updater configuration | Add perception and planning support without enabling MoveIt controller execution |
| `spot_moveit_config/config/spot.srdf` and `spot.urdf` | Arm group is rooted at `body`; robot collision geometry and adjacent-link exclusions exist | Frame handling and self-collision behavior require explicit regression tests |
| `spot_moveit_config/config/ompl_planning.yaml` | OMPL RRTConnect, request adapters, and sampled motion validity checking | Keep the planner initially; validate collision sampling and postprocessing with thin obstacles |
| `application/commanding/semantic_command.py` and `application/ros` | Commands cross an operational-intent boundary and then a semantic-command transport boundary | The UI option must survive both translations |
| `application/recording/semantic_command_codec.py` | Semantic commands are persisted and replayed | Collision policy must have deliberate recording and migration semantics |
| `inspection/execution/saved_probe_motion.py` | Both aligned pre-approach and custom final paths become `FOLLOW_MOVE_TO_TAG_PATH` | Command ID alone cannot decide whether a path is a contact exception |
| `manipulation/move_close_to_surface_execution.py` | Uses guarded Cartesian probe movements and recovery | Apply policy to generated segments and recovery, not only the initial API call |
| `application/controllers/sensor_attachment_controller.py` | Owns confirmation, attachment revision, and reservations | Reuse this owner when head collision geometry is introduced |
| `inspection/model/sensor_models.py` | Persists `hand_to_probe` and exposes immutable attachment snapshots | Extend this model; do not introduce a competing head registry |
| `launch/lidar_rtab_mapping_launch.py` | RTAB-Map uses 3D lidar, `Grid/3D=true`, and a fixed arm exclusion box | Keep mapping behavior; branch MoveIt perception before the box filter |
| `config/nav2_spot_pointcloud_params.yaml` | References front-left and front-right depth point clouds, among others | These are configuration candidates, not proof of live stream availability |

The installed `moveit_msgs` package is 2.2.1. The installed MoveIt header reports `2.5.9-Alpha`. Reconfirm the actual overlay and package versions when implementation begins; upstream's moving `humble` branch is not an exact description of these installed interfaces.

## Decisions after technical review

### Use a small MoveIt capability for request-specific policy

The installed `GetMotionPlan` request contains only a `MotionPlanRequest`. The installed `GetCartesianPath` request has no planning-scene diff and no environmental-only switch. `MoveGroup` with `plan_only` supports a scene diff for ordinary planning, but does not directly replace the existing Cartesian service contract.

Use a small C++ capability loaded into the existing `move_group` process. It reads that process's authoritative planning scene and invokes the existing planning pipeline or Cartesian interpolation against a request-local scene. Keep one Python planning adapter in `MoveItArmPlanner` for both modes. This avoids temporarily modifying shared collision rules or running a second MoveIt server.

Prefer a dedicated `ament_cmake` support package, provisionally `fault_detector_moveit`, for the capability and its C++ tests. Keep `fault_detector_spot` as `ament_python` and `fault_detector_msgs` as interface definitions. Creating that sibling package and editing `spot_moveit_config` exceed the current package-edit boundary in `AGENTS.md`; obtain explicit scope authorization before those implementation edits. No such edits are needed to finish this planning document.

### Remove only the new environment from the local scene

For the initial implementation, the new environment is the occupancy map. For an override request, remove the occupancy object from the private scene through the MoveIt world API using `PlanningScene::OCTOMAP_NS`. Preserve the robot state, attached bodies, existing collision matrix, padding, and any independently defined world objects.

Do not call the shared monitor's `clearOctomap()` to implement an override. Do not switch `avoid_collisions` off. Do not whitelist every robot link against every world object. If later perception produces separate collision objects, give them explicit ownership and add only those objects to the bypass set.

This deliberately defines the override as “ignore the new sensor environment,” not “ignore every world constraint.” No manually modeled keep-out region is silently bypassed.

### Make snapshot ownership explicit

A scene clone alone must not be assumed to deep-copy the live octree. Build a private scene while holding the appropriate scene and octree read locks. For checked requests, deep-copy the occupancy tree and preserve its pose before releasing the locks; for bypass requests, remove it from the private scene before planning. Detach inherited mutable state and ensure private changes cannot invoke live-scene update callbacks.

Reuse MoveIt locking and scene APIs rather than writing another scene store. Test that subsequent occupancy updates cannot change an in-progress request's geometry. Holding the live read lock for an entire multi-second plan is a possible diagnostic baseline, not the intended steady-state design because it blocks perception updates.

### Prove the world frame before adding live perception

The inspected 2.5.9 planning-scene monitor creates the occupancy monitor using the planning frame and installs the resulting tree with an identity transform. Consequently, changing an `octomap_frame` parameter alone is not an adequate solution for the current `body`-rooted setup. [Versioned planning-scene monitor source](https://github.com/moveit/moveit2/blob/2.5.9/moveit_ros/planning/planning_scene_monitor/src/planning_scene_monitor.cpp)

Preferred integration: give the MoveIt model a world planning frame such as `odom` through an SRDF floating virtual joint from `odom` to `body`, with its state supplied by the existing TF chain. Keep the arm group at the existing six arm joints. The floating base is observed, not planned. Existing hand targets may remain expressed in `body`; transform them consistently into the scene.

Prove current-state updates, passive leg state, target transforms, and fixed-base arm planning offline before adopting this change. Do not add another TF broadcaster for transforms already published by Spot. If this route fails with the installed stack, stop at the frame integration milestone and revise the design; do not ship accumulation in a moving frame or silently add a second map owner.

### Retain established planning and mapping algorithms

Use the supported MoveIt occupancy pipeline and current OMPL planner first. Adding GPU planning, a distance-field stack, MoveIt Servo, or another SLAM system would expand scope without resolving the immediate policy, frame, and sensing requirements.

RTAB-Map remains the global mapping and localization owner. MoveIt owns the collision representation used for arm planning. The first version does not ingest RTAB-Map's accumulated map as well as the live streams. A future persistent-map integration needs explicit rules for stale geometry and localization corrections.

## Ownership and data flow

```text
UI movement intent with optional override
  -> application API and intent adapter
  -> semantic command with resolved collision policy
  -> CommandController and existing ROS command transport
  -> Behavior Tree action
  -> ArmMovementExecutor
  -> MoveItArmPlanner
  -> capability in the existing move_group process
       -> private scene derived from the authoritative scene
       -> existing OMPL pipeline or Cartesian interpolation
  -> trajectory and planning metadata
  -> existing executor validation, guards, and Spot transport

Lidar and depth inputs + TF + joint states
  -> MoveIt occupancy updates and existing scene monitor
  -> authoritative planning scene

Existing sensor registry and confirmed attachment revision
  -> scene geometry adapter, introduced later
  -> attached collision object in that same scene
```

The planner capability owns scene preparation and planning validity. The executor owns movement admission, cancellation propagation, execution guards, and physical completion. The UI displays readiness and intent; it does not infer readiness independently. Sensor attachment truth remains in `SensorAttachmentController`.

## Collision policy contract

Use an explicit domain enum rather than scattered booleans. Suggested names are `DEFAULT`, `CHECK_ENVIRONMENT`, and `IGNORE_SENSOR_ENVIRONMENT`. The executor-facing convenience API can expose `avoid_environment_collisions`, but normalize it once into the same domain policy.

`DEFAULT` is allowed at public intent and stored-command boundaries. Resolve it by command or segment purpose before creating a planning request. The planning capability accepts only the two concrete policies. Invalid values fail validation.

During development, a startup feature setting leaves environmental perception disabled and preserves baseline behavior for default commands. An explicit checked request fails if the feature is disabled or not ready. Once enabled, ordinary default motions resolve to checking; sensor failure must never silently turn them into bypass motions. Display the feature-disabled state separately from a one-motion override.

| Motion purpose after feature activation | Proposed resolved policy | Scope |
| --- | --- | --- |
| Basic arm offset, move-to-tag, or orientation movement | Check environment | One command and its normal correction segments |
| Basic movement carrying the one-shot UI override | Ignore sensor environment | Only that command, including corrections implementing the same goal |
| Routine safe approach and aligned pre-approach travel | Check environment | Travel before the contact stage |
| Explicit close-to-surface approach or contact search | Ignore sensor environment | Designated approach segments; retain force, axis, travel, and telemetry checks |
| Fully custom final probe path | Ignore sensor environment | Final path identified by its factory, not every `FOLLOW_MOVE_TO_TAG_PATH` command |
| Retreat immediately out of intentional contact | Explicitly ignore sensor environment where required | Bounded existing retreat; do not extend the exception into unrelated travel |
| Checkpoint recovery and retry | Explicit policy determined by the segment being recovered | Record enough motion context to avoid accidental policy inheritance |
| Base navigation, gripper commands, SDK ready or stow operations | Not covered by this feature unless they actually invoke the planner | Do not advertise environmental protection for bypassing SDK operations |

The proposed contact defaults preserve the requested workflows. Make them visible in command status. Do not silently reclassify any arbitrary failed checked movement as an intentional-contact movement.

Planning-only cancellation must terminate or discard the computation and guarantee that its late result cannot trigger movement. A cancel request is not proof that the planner worker has stopped. Bound compute time and serialize access to planner resources that are not safe for simultaneous requests.

## Development steps

### Step 0 Confirm the implementation boundary and prove feasibility

Read the current checkout and preserve any pending changes. Confirm the runtime overlay, MoveIt versions, model files, and all consumers of the affected ROS messages. Resolve authorization for the proposed C++ support package and MoveIt configuration edits before changing them.

Create isolated proof tests with the installed C++ headers and libraries, without starting ROS nodes or accessing the robot. Prove:

- A private scene can exclude its occupancy object while leaving the shared scene, self-collision rules, and attached bodies unchanged.
- Checked scene snapshots do not share a mutable octree with subsequent updates.
- Existing OMPL and Cartesian APIs can run against the prepared scene with current constraints and timing behavior.
- The proposed world-frame model observes the base transform without adding base joints to the arm plan.

Record actual results in the implementation notes. If a proof fails, resolve it before implementing UI or sensor integration. This is the first coding increment; no sensor mount model is required.

**Exit condition:** a selected, compiled implementation route for both planning modes and a passing scene-isolation/frame proof. No production defaults changed.

### Step 1 Define the domain and transport contracts

Add the collision policy to `SemanticCommand`, relevant execution commands, and the existing `OperationalIntent` and `CommandPayload` boundaries. Keep request identity, origin, recording policy, and sensor identity semantics unchanged. Update `operational_intent_adapter.py`, `semantic_command_adapter.py`, command translation, and factories that construct or replace commands.

Define one planning-only action for the Python-to-MoveIt boundary, provisionally `PlanArmMotion.action`. An action is appropriate because planning is bounded but cancellable and can outlive a client timeout. Keep the schema small:

| Field group | Required information |
| --- | --- |
| Goal identity | Request identity or correlation token, concrete collision policy, planning mode |
| Ordinary planning | Existing `moveit_msgs/MotionPlanRequest` |
| Cartesian planning | A typed specification matching the currently used header, start state, group, link, waypoints, step/jump values, and constraints |
| Consistency | Expected attachment revision when applicable, and the scene epoch required by the execution context |
| Result | Trajectory, MoveIt error code, Cartesian fraction where relevant, detail, scene epoch and observation metadata used |
| Feedback | Waiting for scene, planning, validating, cancelling; no physical execution states |

Use standard MoveIt messages where available. ROS services' request types cannot simply be nested as message fields; define a Cartesian specification message only for the fields actually crossing this boundary. Reject inconsistent mode/payload combinations. Preserve the installed interface's behavior rather than copying newer `humble` fields that are absent locally.

Add `moveit_msgs` and other direct message dependencies to `fault_detector_msgs` only where needed by the new definitions. Rebuild all affected interface consumers together; this is not wire-compatible with stale generated message code.

**Exit condition:** meaningful domain, adapter, and serialization tests cover both policies, invalid values, and defaults without starting nodes.

### Step 2 Implement the planning capability and private scene preparation

Load the capability into the existing planning-only `move_group`. Use its scene monitor and planning pipeline; do not construct another continuously maintained planning scene or execute trajectories in the capability.

Prepare a coherent snapshot of robot state, frame transforms, attachments, collision rules, and occupancy geometry. Apply the requested policy to that snapshot. Keep the live world intact under success, cancellation, timeout, and exception paths.

Run ordinary requests through the existing OMPL pipeline and adapters. For Cartesian requests, use MoveIt's interpolation and validity APIs with collision checking enabled on the prepared scene. Preserve current complete-path requirements, jump behavior, limits, trajectory timing, and Python-side validation. Where code must adapt the upstream capability, keep that adaptation narrow and tied to the installed version. [Versioned Cartesian capability](https://github.com/moveit/moveit2/blob/2.5.9/moveit_ros/move_group/src/default_capabilities/cartesian_path_service_capability.cpp)

Cancellation and timeouts must suppress results for superseded operations. Do not start a new request against shared non-thread-safe planning resources while a previous computation is still finishing. Keep cancellation callbacks responsive independently of the planning worker.

Validate the final returned trajectory after planning adapters and time parameterization using the same policy. Collision sampling must cover interpolation between output points, not just the listed waypoints. Test thin obstacles and account for the interpolation used when the existing Spot transport executes the trajectory. Do not claim continuous swept-volume guarantees from sampled checking.

**Exit condition:** synthetic obstacles block or reroute checked normal paths, truncate/reject checked straight paths, and do not block explicit bypass paths. Self-colliding paths fail in both modes. The live scene remains unchanged by every override.

### Step 3 Integrate executor and workflow policy

Adapt `MoveItArmPlanner` to the new planning endpoint for both modes. Keep the existing trajectory result contract where possible. Update the shared construction in `robot_command_resources.py`; avoid creating separate planners for checked and bypass movements.

Thread policy through `ArmMovementExecutor` operations and the existing command/Behavior Tree adapters. Freeze it for each active operation. Clear operation-local state on success, failure, cancellation, timeout, and destruction.

Review all generated movements: ordinary position corrections, path waypoints, guarded Cartesian approach steps, contact retreat, checkpoint restoration, and retry preparation. In particular, `saved_probe_motion.py` must distinguish the custom final path from aligned pre-approach despite their shared command ID. The composite probe workflow must assign policy to each stage rather than copying a blanket exception over the whole routine.

Keep stop confirmation and existing contact telemetry requirements unchanged. Existing recovery paths that bypass MoveIt remain explicitly outside the new coverage; converting them is separate work. A checked planning failure must not fall back to direct Cartesian execution.

Make status report the effective policy and a useful failure category: perception unavailable, stale state, inconsistent scene/attachment, collision-related planning failure, incomplete Cartesian path, timeout, or cancellation. Do not label every failed plan as a collision when MoveIt reports only a general failure.

**Exit condition:** executor and workflow tests show correct policy for every affected segment and no policy leakage into the next operation. Existing guards and stop behavior still pass.

### Step 4 Establish a stable scene frame and lifecycle

Implement the proven world-frame configuration from Step 0, keeping the arm group and probe-to-hand conversion unchanged. Test a fixed obstacle while the body translates, rotates, or changes height, plus recorded body sway and passive leg updates. The obstacle must remain fixed in the world and move correctly relative to the robot model.

Track a scene epoch for discontinuities: startup, deliberate clear/rebuild, odometry reset, sensor configuration change, and attachment geometry replacement. This is not the same as a normal occupancy update counter. Reject old plans after an epoch change.

For the first release, bound the local map by resetting and rebuilding it between stationary manipulation sessions after navigation. Do not clear it during an active movement. Do not assume the standard OctoMap updater implements a rolling window or timed voxel expiry. Later pruning is separate work if measured memory or ghost obstacles require it.

At dispatch, require current joint state, acceptable base pose stability, the expected scene epoch, and compatible attachment revision. A check immediately before sending a trajectory reduces stale-plan risk but is not execution-time obstacle monitoring.

**Exit condition:** offline transform fixtures show no body-frame accumulation errors, and all epoch-invalidating events reject stale checked plans.

### Step 5 Add one lidar input with trustworthy readiness

Configure the MoveIt occupancy updater using `/velodyne/points` as the current candidate input, before the existing mapping arm-box filter. Verify actual schema, frame ID, timing, and QoS from offline data or later authorized observation. Do not change the RTAB-Map filter as part of this work.

Keep the lidar driver lifecycle separate from mapping. Attaching hardware alone does not provide a cloud: the configured driver must be running when live operation is eventually authorized. The MoveIt integration must not start mapping to obtain lidar data.

Apply robot self-filtering using the current model. Separate self-filter padding from collision clearance: a larger exclusion region can erase actual nearby obstacles. A missing mount model is a declared geometry limitation; it is not a reason to increase the arm exclusion box.

Choose local range, resolution, update rate, and padding from measured noise, the smallest obstacle of concern, and planning latency. A 2 cm voxel size is a possible bench starting point, not an acceptance value or a guarantee of millimetre contact clearance. Keep the existing dense Cartesian sampling until evidence supports a change.

Expose one authoritative perception status to the application. It must distinguish feature-disabled, warming-up, ready, stale, and invalid states. Track sensor acquisition time, receipt age, successful transform/filtering, and successful map integration separately. A recent message, filtered-cloud debug publication, or generic scene update is not sufficient evidence that useful measurements entered the occupancy map.

During implementation, identify the installed updater's exact integration callback. If it cannot attribute successful updates per required input, add the smallest updater adapter/instrumentation necessary in the support package; retain the standard occupancy algorithm. Do not replace the scene monitor's own callback or invent successful integration from an unrelated joint-state update.

A checked request requires recent integrated data for the configured required sensors, valid TF and robot state, a built scene in the current epoch, and operation within the validated sensing workspace. An empty/all-invalid cloud does not establish clear space. Set explicit startup and freshness timeouts; determine their deployment values from observed rates and processing delay. No automatic bypass on timeout.

**Exit condition:** synthetic or directly decoded recorded clouds produce the expected occupied geometry and self-filtering; missing, stale, empty, malformed, or untransformable input prevents checked planning. Explicit bypass remains independent of environmental perception while retaining baseline robot-state requirements.

### Step 6 Implement the one-shot basic-movement UI option

Add the option to `ui/manipulation/controls.py` with text such as “Ignore environmental obstacles for next basic movement.” Initially define basic movements as arm offset, move-to-tag, move-to-tag-and-wait, orient-to-tag, and orient-to-surface. The wait part does not consume another override. Base commands, ready/stow, gripper actions, and saved inspection workflows do not consume this UI option.

The existing `execute_operation()` returns a local submission ID before asynchronous acceptance. Extend `ui/ros/application_client.py` to expose correlated submission acceptance/rejection to the controls; do not treat the local ID as server acceptance.

Reserve the armed override for the clicked command while its submission is pending, preventing double-clicks from assigning it twice. Consume it after confirmed semantic admission. A definite pre-admission rejection restores the pending choice if the UI context is unchanged. An uncertain submission result must not automatically rearm it; display the uncertainty and reconcile using existing request correlation. An admitted command that later fails or is cancelled still consumes the override.

Unrelated clicks must neither consume nor inherit it. Clear unsent UI intent on disconnect, closing the control, or switching out of its context. Display the effective policy for the active or queued command separately from the next-command toggle. Do not add a repeated confirmation dialog.

Record the explicit resolved policy with accepted basic commands so playback does not silently change the meaning of a deliberately bypassed move. Existing recordings missing the field deserialize to `DEFAULT`, using the currently configured purpose-based policy; do not guess a historical bypass. Composite inspection recordings preserve workflow policy through their existing stage factories. The UI checkbox itself is never persisted as an armed global state.

**Exit condition:** UI/client tests cover asynchronous acceptance, rejection, unknown outcome, double-click, unrelated command, queued command, failure after acceptance, and recording/playback. No physical command is sent by a test.

### Step 7 Add front and hand depth sources incrementally

Add front-left, then front-right, then hand depth. Use actual depth images with calibration or valid point clouds; a colour image is not an occupancy input. Discover and verify the hand stream rather than inventing its topic name.

Give each source its own updater input and retain the physical sensor origin for free-space ray tracing. Do not concatenate independently located clouds into one cloud and pretend they share an origin. Check registered-cloud frame semantics as well as frame names.

For each source, compare common surfaces, self-filtering during arm motion, near-range validity, latency, and CPU load. Test high-rate noisy camera measurements against slower lidar returns: standard fusion does not automatically assign the lidar greater authority. Tune source rates, noise rejection, range, and field-of-view cropping based on those results.

Define named, validated sensor configurations with required and optional inputs. Begin with lidar required. Adding cameras as optional does not justify a wider claimed workspace until their coverage is validated. A later camera-only configuration can work without lidar, but must pass its own readiness and coverage tests. Mapping state is never the readiness switch.

**Exit condition:** combined inputs do not create persistent duplicate surfaces or erase known obstacles in the tested workspace, and required-source failure produces an explicit unavailable state.

### Step 8 Add optional sensor-head and mount geometry

This increment may follow the first lidar release; its absence must be visible as limited modeled geometry. Correct perception sensor TF remains mandatory throughout.

Extend `SensorDefinition` with optional collision shapes or a geometry-profile reference owned by the existing sensor repository. Define shapes relative to the existing probe frame when appropriate and compose with the authoritative `hand_to_probe` transform. Do not duplicate calibration in another YAML owner. A shape's local offset is geometry, not a new hand-to-probe calibration.

Start with boxes/cylinders and later support simplified meshes if needed. Example shape metadata inside an existing sensor definition could express a box in the probe frame, its dimensions, and a local pose; actual dimensions must come from the physical head. Validate units, finite positive dimensions, mesh scale, and transform conventions.

Use a scene adapter to attach the selected head to `hand`. Replace it atomically on a confirmed attachment revision, with only the necessary mounting links listed as touch links. Do not allow the head to collide with the whole arm. Ensure the updater excludes attached geometry as well as URDF geometry, and test old occupied cells after replacement.

Tie synchronization to the existing confirmation and reservation lifecycle. A head-dependent movement must not start until the scene acknowledges the expected geometry revision. Persistent body-mounted equipment can use reusable URDF/Xacro components; different removable heads do not need complete robot URDF copies.

Attached geometry remains present in bypass mode, including head-versus-robot checking. An absent optional profile means “unmodeled attachment,” not “no sensor physically attached.” Do not change sensor attachment confirmation just to accommodate missing collision shapes.

**Exit condition:** head switching, bare hand, missing profile, and stale revision work predictably; self-collision and environmental checks include modeled attachments appropriately; calibration and probe-axis tests still pass.

### Step 9 Complete offline verification and performance checks

Run relevant existing Python tests, new domain/adapter/UI tests, and in-process C++ planning tests. Use isolated temporary build/install/log directories where possible. Build the affected packages and consumers after ROS interface changes.

Offline validation must not depend on a running production ROS graph. Decode available bag data directly or use fixture messages for perception tests. ROS launch tests, bag playback that starts ROS nodes, and robot-connected checks require separate explicit approval under the working preferences; an isolated test label does not override that rule.

Measure snapshot copy cost, local-map size, occupancy update latency, planning duration, cancellation latency, and final trajectory validation cost. Keep the work bounded to a local workspace. Set acceptance limits before deployment from the existing command timeouts and measured platform performance; do not increase timeouts merely to hide unbounded work.

Relevant existing suites include:

- `test_moveit_arm_planner.py`, `test_moveit_joint_trajectory_execution.py`, `test_arm_movement_executor.py`, `test_guarded_arm_movement_executor.py`, and `test_arm_movement_behaviour.py`.
- `test_command_controller.py`, `test_command_request_adapter.py`, `test_execution_command_translation.py`, `test_semantic_command_boundary.py`, and `test_semantic_record_manager.py`.
- `test_execute_probe_point.py`, `test_move_close_to_surface_command.py`, and the sensor attachment/model/geometry tests affected by Step 8.

Add behavior tests that protect the new contracts. Do not replace meaningful production checks with test-double shortcuts or add tests that merely repeat assignments.

**Exit condition:** the acceptance matrix below passes at the applicable offline level, with untested physical coverage and performance assumptions explicitly recorded.

### Step 10 Validate and activate on the robot after approval

Request explicit approval for the concrete node starts and physical validation sequence only when the implementation and offline evidence are ready. Do not deploy or restart services as part of preparing the change.

The proposed physical sequence is stationary observation first, then plan inspection without execution, then low-speed checked free-space movements, then controlled intentional-contact/bypass trials under existing guards. Test each validated sensor configuration, sensor dropout, head changes if implemented, and returning to manipulation after navigation.

Inspect both the planned trajectory and observed execution. Verify start-state agreement and actual clearance, including arm links rather than just the tip. Stop activation if frame alignment, self-filtering, or unknown-space coverage is insufficient.

Enable default environmental checks only after the corresponding acceptance evidence exists. An explicit startup rollback setting restores pre-feature defaults and reports environmental checking as disabled; it must never be an automatic reaction to planning failure. Remove or disable stale capability configuration coherently rather than leaving mismatched message consumers.

**Exit condition:** a documented validated operating envelope and an explicit activation decision. SDK motions outside the planner remain labeled outside coverage.

### Step 11 Add execution-time obstacle monitoring as a separate extension

If changing obstacles during arm execution must be handled, add remaining-trajectory validation against fresh scene snapshots and stop requests through the existing executor/transport. Carry the same collision policy into that monitor so intentional contact is not immediately rejected by a second checker.

Time-critical stop decisions must not depend on Behavior Tree ticks. Measure sensing delay, validation delay, command latency, and physical stopping behavior before defining reaction margins. Replanning may follow confirmed stopping; it must not silently change a commanded straight probing path into a detour.

Do not describe Steps 0–10 as providing this functionality. Existing force guards remain contact reactions, not predictive visual obstacle monitoring.

## Acceptance matrix

| Case | Required result | First verified in |
| --- | --- | --- |
| Clear scene, normal and Cartesian requests | Existing constraints, timing, and trajectory checks retained | Steps 0–3 |
| Occupancy obstacle across normal route | Detour or a useful planning failure | Step 2 |
| Occupancy obstacle across straight Cartesian route | Incomplete path rejected; no detour or partial execution | Step 2 |
| Same occupancy with explicit bypass | New occupancy does not block; existing checks remain | Step 2 |
| Self-collision under either policy | Rejected | Step 2 |
| Independently modeled world obstacle under bypass | Still checked | Step 2 |
| Scene updates while checked planning runs | Coherent private geometry; no data race or live mutation | Steps 0–2 |
| Cancel, timeout, or late planner result | No movement and no policy leakage | Steps 2–3 |
| Custom final versus pre-approach path with same command ID | Different intended policies survive factory and transport | Step 3 |
| Contact retreat and checkpoint recovery | Bounded, explicit policy; stop prerequisites unchanged | Step 3 |
| Base translation, rotation, height change, or odometry reset | Correct geometry alignment or rejection and rebuild | Step 4 |
| Missing, invalid, stale, or all-empty required input | Checked motion unavailable; no silent bypass | Step 5 |
| Recent joint-state update but no occupancy integration | Perception does not become ready | Step 5 |
| Required sensor absent while mapping is active | Unavailable despite mapping state | Step 5 |
| Mapping stopped with valid configured live sensing | Checked planning can remain available | Step 5 |
| Thin obstacle and final trajectory interpolation | Rejected at the configured checking resolution | Steps 2 and 9 |
| One-shot UI acceptance and rejection races | Exactly one admitted command receives the override | Step 6 |
| Recorded explicit bypass and older recording | Explicit policy round-trips; missing field uses documented default | Step 6 |
| Overlapping cameras and lidar | No unexplained obstacle clearing or persistent misalignment | Step 7 |
| Attachment revision changes | Old plan invalidated; new geometry synchronized | Step 8 |
| Unmodeled mount | Limited coverage displayed; no invented geometry guarantee | Steps 5 and 8 |
| Obstacle appears after execution starts | Outside initial guarantee; handled only after Step 11 validation | Step 11 |

## Implementation areas and scope

| Area | Planned changes |
| --- | --- |
| `fault_detector_spot/application/commanding` | Policy type, semantic command validation, explicit resolution by purpose |
| `fault_detector_spot/application/ros` and `application/api` | Intent and command adapters; existing operation/status boundary integration |
| `fault_detector_spot/application/recording` | Serialize policy and handle existing saved commands |
| `fault_detector_spot/application/behaviour_tree` | Command translation and propagation, cancellation, shared planner construction |
| `fault_detector_spot/manipulation` | Planning action client, executor policy, guarded surface segments and recovery |
| `fault_detector_spot/inspection/execution` and `inspection/behaviours` | Stage-specific policy for saved and composite probe workflows |
| `fault_detector_spot/ui/manipulation` and `ui/ros` | One-shot option, correlated acceptance, readiness presentation |
| `fault_detector_spot/inspection/model`, repositories, attachment controller/adapter | Optional geometry extension in Step 8, reusing existing ownership |
| `fault_detector_spot/config`, `launch`, and `test` | Feature settings, integration wiring, offline fixtures and tests |
| `fault_detector_msgs` | Policy fields, planning action/specification, status boundary only where needed |
| Proposed `fault_detector_moveit` | Small C++ capability, private-scene helpers, integration evidence, in-process tests; requires scope authorization |
| `spot_moveit_config` | Frame, capability, and sensor parameters; requires scope authorization |

Do not refactor unrelated command, mapping, UI, or geometry code. Keep the existing RTAB-Map filtering and map lifecycle unchanged unless a later, separate requirement identifies a necessary modification.

## Review findings and adjustments

| Earlier proposal or potential shortcut | Review outcome and adopted adjustment |
| --- | --- |
| Pass one boolean directly to current MoveIt services | Insufficient for selective checking. Add a request-local capability and keep collision checks enabled |
| Change the shared collision matrix for one command | Rejected because other requests, cancellation, and exceptions can observe the changed state |
| Run a second MoveIt server without obstacles | Rejected because it duplicates maintained scene/state ownership and creates divergent planning paths |
| Replace normal planning with the standard plan-only action alone | Useful for ordinary requests, but leaves the Cartesian requirement unresolved; use one narrow provider for both |
| A scene clone is an immutable snapshot | Not assumed. Explicitly isolate mutable occupancy data and verify updates cannot affect it |
| Set `octomap_frame=odom` while leaving the model unchanged | Not accepted as a fix for the inspected version. Prove the world-frame model and monitor integration first |
| Use the existing filtered mapping cloud | Avoid the arm box for manipulation; retain geometry-based self-filtering and mapping's current behavior |
| Enable readiness when clouds or any scene updates arrive | Require evidence of usable integrated measurements and the current scene epoch |
| Reset the UI toggle when `execute_operation()` returns an ID | The return is asynchronous submission, not admission. Correlate acceptance and handle uncertain outcomes |
| Detect contact intent from `FOLLOW_MOVE_TO_TAG_PATH` | Incorrect because pre-approach and custom final paths share that ID. Assign purpose in factories |
| Add a new sensor-head registry | Reuse the existing sensor definition, confirmation, revision, and reservation mechanisms |
| Adding an occupancy map also handles moving obstacles during execution | It does not. Keep execution monitoring as a separate, measured extension |

The approach fits the project when implemented in this order. The largest uncertainties are the world-frame integration, correct private octree ownership, and observed multi-sensor coverage. They have early proof or acceptance steps rather than being deferred until deployment. The UI and optional head geometry should not delay resolving those foundational issues.

## Reference points

Local files are the authority for the installed interface contract:

- `/opt/ros/humble/share/moveit_msgs/srv/GetMotionPlan.srv`
- `/opt/ros/humble/share/moveit_msgs/srv/GetCartesianPath.srv`
- `/opt/ros/humble/share/moveit_msgs/msg/PlanningOptions.msg`
- `/opt/ros/humble/share/moveit_msgs/action/MoveGroup.action`
- `/opt/ros/humble/include/moveit/version.h`
- `/opt/ros/humble/include/moveit/planning_scene/planning_scene.h`
- `/opt/ros/humble/include/moveit/planning_scene_monitor/planning_scene_monitor.h`
- `/opt/ros/humble/include/moveit/move_group/move_group_capability.h`

The versioned sources below support the implementation review; compare them with the installed build before adapting code:

- [MoveIt 2.5.9 planning scene implementation](https://github.com/moveit/moveit2/blob/2.5.9/moveit_core/planning_scene/src/planning_scene.cpp), including scene cloning, occupancy geometry, and attached objects.
- [MoveIt 2.5.9 planning scene monitor](https://github.com/moveit/moveit2/blob/2.5.9/moveit_ros/planning/planning_scene_monitor/src/planning_scene_monitor.cpp), including scene/occupancy locking and frame integration.
- [MoveIt 2.5.9 Cartesian capability](https://github.com/moveit/moveit2/blob/2.5.9/moveit_ros/move_group/src/default_capabilities/cartesian_path_service_capability.cpp), including collision validity callbacks and timing.
- [MoveIt 2.5.9 occupancy monitor](https://github.com/moveit/moveit2/blob/2.5.9/moveit_ros/occupancy_map_monitor/src/occupancy_map_monitor.cpp), for multiple updater inputs and their integration with the monitor.
- [MoveIt Humble perception overview](https://moveit.picknik.ai/humble/doc/examples/perception_pipeline/perception_pipeline_tutorial.html), for the supported depth/point-cloud approach. Some examples retain older launch syntax; use installed ROS 2 interfaces and parameters for implementation.
