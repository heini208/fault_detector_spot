# MoveIt collision avoidance implementation plan

Add optional environmental collision checking to the existing arm planner. Its purpose is to reduce avoidable collisions and failed movements before execution. The existing collision guard continues to react during movement. This is a best-effort planning aid, not a replacement safety system.

Keep the current motion generator, endpoint tolerances, calibration, speed settings, trajectory validation, and execution path. When this layer is disabled, sensor obstacles must not affect planning. When enabled, they may change the route or prevent a plan, but must not change the requested target or relax accuracy requirements.

This simplified plan supersedes the earlier custom-capability, private-snapshot, and new-action proposal. It reflects the clarified requirements of 7 October 2026.

## Keep the existing architecture

The command path stays:

**UI or inspection workflow → command infrastructure and Behavior Tree → ArmMovementExecutor → MoveItArmPlanner → existing MoveIt service → existing validation and execution guard.**

The perception path is:

**Lidar and later depth cameras → standard MoveIt occupancy updater → local obstacle map in the existing planning scene.**

Continue using **GetMotionPlan** for ordinary planning and **GetCartesianPath** for straight approaches. Keep **avoid_collisions=True**: switching it off would also remove self-collision checks.

Use MoveIt's existing Allowed Collision Matrix to check or ignore collisions with the occupancy map. Preserve self-collision rules and independently modeled world objects. No new motion planner, C++ package, planning action, second MoveIt server, or task framework is needed for this proposed first version.

The application already serializes commands through CommandController and shares one MoveItArmPlanner. Use that planning lane rather than designing for independent concurrent planning clients.

## Preserve current motion behavior

| Existing behavior | Requirement |
| --- | --- |
| Target calculation and probe-to-hand transforms | Unchanged |
| Position and orientation tolerances | Unchanged |
| Cartesian interpolation, jump checks, and required completion fraction | Unchanged |
| Joint limits and trajectory validation | Unchanged |
| Speed scaling and trajectory timing | Unchanged |
| Contact detection, stopping, retreat, and force limits | Unchanged |
| Physical command ownership | Existing executor only |
| RTAB-Map mapping and its arm exclusion box | Unchanged |

An enabled obstacle check can produce a different route, reject a path, or add planning latency. That is its intended effect. A disabled check must preserve existing planning rules; identical sampling-based trajectories are not guaranteed. Verify endpoint accuracy and baseline success cases through regression tests.

## Simple toggle behavior

Use one command option, provisionally **ignore_environment_collisions**, defaulting to false when the feature is enabled. The executor passes the effective choice to the planner. Resolve the overall feature setting and workflow-specific choices before planning. A UI change does not modify an in-flight motion.

| Motion | Default after enabling the feature |
| --- | --- |
| Basic arm movement and ordinary approach travel | Check the sensor environment |
| Basic movement with the one-shot override | Ignore the sensor environment for that command |
| Designated close-to-surface movement or contact search | Ignore sensor obstacles; retain existing guards |
| Fully custom final probe path | Ignore sensor obstacles for the designated final path |
| Travel to safe or aligned pre-approach positions | Check the sensor environment |
| Contact retreat and recovery | Preserve existing execution behavior; explicitly carry the appropriate choice into MoveIt-planned segments |

The option does not disable self-collision checking, bypass execution guards, or affect base navigation. SDK ready/stow and recovery movements that bypass MoveIt do not gain environmental checking from this feature.

Keep the requested UI option: **Ignore environmental obstacles for next basic movement**. An overall feature setting can disable the layer for a session. A separate live global-settings workflow is unnecessary initially.

## Apply the policy before each plan

This is the key simplification:

1. Read the current collision matrix using GetPlanningScene.
2. Change only occupancy-map-to-robot collision entries, including relevant attached bodies when modeled. Preserve all other entries and defaults.
3. Apply the matrix using ApplyPlanningScene and wait for success.
4. Submit the existing normal or Cartesian planning request.
5. Process the result as today.

Use the installed MoveIt's occupancy object identifier and verify the matrix transformation with synthetic tests. Explicit pair entries take precedence over defaults; changing only a default is not necessarily sufficient.

The policy can remain set after planning. Every following plan explicitly establishes its own setting before submission, so there is no temporary disable/restore sequence. Do not delete the map to implement the toggle. The map can update while ignored. The standard apply-scene service acknowledges whether the update succeeded. [Versioned apply-scene implementation](https://github.com/moveit/moveit2/blob/2.5.9/moveit_ros/move_group/src/default_capabilities/apply_planning_scene_service_capability.cpp)

This assumes the application is the only active planning client and collision-matrix writer for this move_group. RViz may visualize the scene, but independent RViz planning must not overlap application planning. If concurrent independent clients become necessary, revisit request-local policy then.

Cancellation needs one targeted change: cancelling a Python future does not prove that server computation stopped. Retain the outstanding service request until it completes, discard cancelled results, and prevent another policy change/plan meanwhile. Do the same for an outstanding scene update. If completion cannot be established, report the planner unavailable instead of guessing. Extend the existing planner lifecycle; do not build another workflow manager.

## Use a short-lived local obstacle map

Keep the current **body** planning frame and robot model in the first version. Do not introduce a floating virtual joint or change the arm's kinematic configuration for this feature.

This makes the obstacle map a temporary local view for stationary arm planning, not a world map accumulated during walking. Before a checked movement, clear the old occupancy map, collect a short bounded batch of fresh observations, and plan. A stationary multi-segment approach may reuse recent observations within a short age limit instead of rebuilding for every small step.

Invalidate the view after walking, deliberate body repositioning, a relevant TF discontinuity, or an expired observation interval. Restrict checked planning to stationary operation. Check whether normal body sway produces unacceptable ghosting; if it does, address that specific frame issue before enabling this operating mode. The guard does not correct misregistered perception.

Use the standard clear-map service and updater, not ROS node restarts. Require a fresh usable source cloud, valid transforms, and a subsequent occupancy update before reporting the map available. Use a bounded wait and a simple unavailable reason. This is coarse freshness checking, not a coverage-certification system.

If sensing is unavailable, the user can explicitly disable the layer and use existing guarded behavior. Do not silently report collision checking as successful with an empty or stale map.

The tradeoff is deliberate: short-lived body-frame sensing is less consistent than a world-fixed map, but avoids changing the existing robot model. If it is unsuitable in practice, leave the feature disabled while resolving the observed limitation.

## Development steps

### Step 1 Prove the toggle using existing services

Add a small collision-policy helper to MoveItArmPlanner using get/apply scene services. Keep it nonblocking. Add only the preparation stages needed before the current planning request; preserve request parameters and result processing.

Verify with synthetic occupancy geometry:

- Checked normal planning avoids the obstacle or fails.
- Checked straight Cartesian planning fails or returns an incomplete path, handled as today.
- An explicit override ignores that occupancy obstacle.
- Self-collision rules and independently modeled obstacles still apply.
- A checked command following an ignored command checks obstacles again.
- Cancelled results cannot execute or affect a later plan.

Start with offline matrix-transformation and planner-client tests. Node-based service integration requires separate approval. No sensor hardware or head geometry is needed yet.

**Done when:** both current planning modes support the toggle without changing their motion-generation contracts.

Implementation status: the optional planner-client policy and offline tests are
implemented. Construct `MoveItArmPlanner` with
`environment_collision_policy_enabled=True` to prepare the shared ACM before
each request; `start` and `start_cartesian` accept
`ignore_environment_collisions=True` for occupancy bypass. Existing construction
keeps the direct planning path and original cancellation behavior. Command/UI
wiring and sensor ingestion are not connected yet. Offline tests cover effective
ACM rules, unchanged planning requests, response handling, and cancellation;
actual avoidance of synthetic geometry through running MoveIt services remains
an integration check requiring separate approval. Use one serialized client and
ACM writer, as described above.

Review and passive hardware check, 7 October 2026, on `feature/move_it_collision`
at `0e4ddbe` plus the Step 1 working changes:

- No Step 1 implementation defect found; 110 relevant offline tests pass.
  A separate experiment against installed MoveIt validated 3,920 ACM comparisons
  and native FCL collision checks: occupancy bypass retained self-collision and
  named-obstacle checks. This did not exercise running planning services.
- The constructor flag enables policy handling; it is not the future UI switch.
  Once sensor occupancy is connected, runtime OFF must still prepare the policy,
  using `ignore_environment_collisions=True`. Skipping policy handling would
  leave existing occupancy rules active.
- A roughly 12-second passive sample received lidar at 6.8 Hz and hand/front-left/
  front-right clouds at 2.6/3.2/3.4 Hz. Median receipt ages were about
  0.45/0.58/0.33/0.33 seconds. All sampled cloud frames resolved to `body` at their
  acquisition timestamps. Joint states arrived at about 39.5 Hz.
- The inspected camera clouds contained about 8.1%/2.8%/3.2% finite nonzero points.
  This establishes data availability, not sufficient obstacle coverage; check
  coverage again in the working arm posture. Lidar contained about 25,000 valid
  points in the inspected cloud.
- Live TF confirmed `sensor_origin_velodyne-point-cloud` is identical to `odom`,
  not a physical lidar-origin frame. Resolve the acquisition-origin requirement
  below before connecting this source to the standard updater.
- No `move_group` or MoveIt planning/scene services were present in the observed
  ROS graph. End-to-end avoidance was not validated. Only temporary passive
  observers ran; no motion, scene changes, or driver restarts were requested.

### Step 2 Carry the option through existing commands

Add the field to the semantic command and relevant existing ROS intent/payload messages. Update producers, adapters, execution-command translation, Behavior Tree dispatch, and executor calls. Keep request identity and correlation unchanged.

Assign contact exceptions where the workflow knows a segment's purpose. In particular, saved_probe_motion.py uses FOLLOW_MOVE_TO_TAG_PATH for both aligned pre-approach and custom final paths. Do not infer a contact exception from that command ID alone. Carry the chosen behavior into corrections and retries belonging to the same motion.

Record the option with accepted commands. Existing recordings missing it use the documented default, with contact exceptions supplied by existing workflow factories. Do not persist the armed UI checkbox.

Keep defaults in the existing parameter system. No new policy enum, command queue, or settings registry is needed for this boolean.

**Done when:** the option reaches both planning modes and current contact/custom probe workflows remain usable.

### Step 3 Connect one sensor and local map refresh

Configure the standard point-cloud occupancy updater in the existing MoveIt launch. Start with the lidar's **/velodyne/points** candidate topic. Branch before the mapping arm-box filter, which can also remove actual objects near the arm. Keep the mapping branch unchanged; use MoveIt's robot self-filter for planning.

Mapping need not run, but the lidar driver must supply usable data. Verify topic, frame, timestamps, QoS, and self-filter behavior. Do not start mapping just to obtain a planning cloud.

The current lidar cloud is expressed at the odometry origin. MoveIt's standard
updater uses the cloud frame origin for free-space rays, so a valid TF alone is
insufficient. Before using this topic, obtain the actual calibrated acquisition
origin and transform the point coordinates into that sensor frame at the cloud
timestamp; changing only `frame_id` is incorrect. Reuse existing TF/calibration
and leave the mapping stream unchanged. If the physical origin is unavailable,
start with one camera cloud after verifying useful coverage instead of assuming
the lidar topic is immediately suitable. See the
[versioned updater implementation](https://github.com/moveit/moveit2/blob/2.5.9/moveit_ros/perception/pointcloud_octomap_updater/src/pointcloud_octomap_updater.cpp).

Implement the bounded refresh described above. Choose a modest local range and voxel resolution for avoiding obvious obstacles. Tune for useful coverage and low overhead rather than millimetre surface reconstruction. Do not increase self-filter padding until nearby real obstacles disappear.

Do not change target tolerances or movement speeds to compensate for noisy occupancy. Fix the input/filtering or disable the optional layer for that motion.

**Done when:** nearby obstacles appear usefully, the arm is filtered, and sensor unavailability is reported clearly. Bypass behavior still matches the baseline.

### Step 4 Add the one-shot basic movement UI option

Add the checkbox to the basic arm controls and include its value in the submitted intent. Reserve it for that submission so a double-click cannot apply it twice. Consume it after confirmed command admission using existing request correlation.

A known pre-admission rejection leaves the choice available. An uncertain submission must not automatically reuse it. An accepted command consumes it even if movement subsequently fails.

Unrelated base, gripper, wait-only, or saved-workflow commands do not consume it. Display whether the active command is checking environmental obstacles. Avoid confirmation dialogs or a separate global UI state machine.

Changing the checkbox while a movement is active applies to a future command, not the current trajectory.

**Done when:** exactly the intended next basic movement receives the override and later movements use their own settings.

### Step 5 Add cameras where they improve coverage

Add front-left, front-right, and hand depth inputs individually. Verify actual depth topics and calibrated transforms. A colour image or an available camera alone is not usable depth.

Use separate updater inputs with their actual sensor origins and one shared occupancy map. Check overlapping surfaces, moving-arm self-filtering, and obvious object insertion/removal. Invalid depth must not be treated as free space. Check that a misaligned/noisy camera does not repeatedly clear a real lidar obstacle.

There is no need to put these cameras into RTAB-Map. Include only sources that help the arm workspace. If the rear lidar cannot see the useful front workspace, bring the appropriate front camera into the first release.

**Done when:** each added source improves useful coverage without persistent false obstacles or excessive planning delay.

### Step 6 Add optional mount geometry later

Initial development does not require housing or mount collision geometry. Unmodeled geometry remains outside modeled protection and may leave sensor returns in the map. Correct TF for perception sensors is required immediately.

Later, extend the existing SensorDefinition with optional boxes/cylinders or a mesh reference. Reuse hand_to_probe and the existing attachment controller. Publish an attached collision object for the confirmed head, with only the necessary mounting links allowed to touch it.

Do not create a second registry or a complete robot URDF per sensor head. Keep modeled attachment geometry active even when the sensor environment is ignored.

**Done when:** the selected head follows the hand and participates in collision checks without altering probe calibration or target calculation. This step does not block the initial feature.

## Focused verification

Use relevant existing planner, executor, guarded-movement, adapter, recording, and probe workflow tests. Add tests only for changed contracts.

| Case | Expected result |
| --- | --- |
| Feature off or explicit override | Current planning parameters, accuracy requirements, self-collision rules, and guards preserved |
| Feature on with a blocking obstacle | Normal route changes or fails; straight Cartesian path remains straight and fails/incompletes if blocked |
| Off followed by on | No leftover exception |
| Collision-policy apply fails | No plan submitted under an unknown policy |
| Cancel/timeout while server still works | Result discarded; no overlapping policy change |
| Contact/custom final path | Existing guarded movement remains possible |
| Sensor unavailable | Clear unavailable state; explicit off mode remains usable |
| Base moved since sensing | Old local view rebuilt before checked planning |
| UI and recording | Choice applies to the intended command and survives serialization |
| Combined sensors | Useful obstacles retained, arm filtered, no obvious misalignment |

For approved physical checks, compare representative target motions with the layer off and on. Measure endpoint error, movement success, unnecessary planning rejection, and added delay. The goal is fewer avoidable contacts with unchanged target accuracy, not a perfect reconstructed environment.

Do not weaken a guard, relax a target tolerance, replace interpolation, or alter speed defaults to pass these checks. No node starts, deployment, or robot tests occur without explicit approval.

## Scope and delivery

The first useful version is Steps 1–4 with one useful sensor. Step 5 expands coverage; Step 6 is optional.

Expected changes are limited to the existing planner/executor, command models and adapters, inspection factories, recording codec, basic UI, sensor configuration, and relevant tests. ROS fields stay in fault_detector_msgs; Python behavior stays in fault_detector_spot.

Loading sensor parameters into spot_moveit_config may require a small launch/config edit. That sibling package is outside the current AGENTS.md edit boundary and needs scope authorization when implementing. No new package is proposed.

Leave out custom planning actions, private scene cloning, per-plan scene epochs, new C++ capabilities, floating-base model changes, alternate planners, automatic replanning, continuous execution monitoring, formal coverage certification, and GPU mapping. Revisit an individual item only if a concrete limitation requires it.

## Why this simpler approach fits

The earlier plan optimized for concurrent clients, strong scene consistency, and future reactive avoidance. Those exceed the clarified requirement. A shared policy set before each plan is a reasonable compromise for the existing serialized application, provided the single-client assumption and outstanding-request handling are enforced.

Keeping the same planner, Cartesian service, robot model, and executor minimizes changes that could affect accuracy and reliability. The disposable local map is only an additional planning input. It can cause false rejections or extra delay when enabled; the toggle explicitly restores baseline planning behavior.

MoveIt's standard perception and scene services provide the necessary mechanisms. [Perception overview](https://moveit.picknik.ai/humble/doc/examples/perception_pipeline/perception_pipeline_tutorial.html), [versioned planning service](https://github.com/moveit/moveit2/blob/2.5.9/moveit_ros/move_group/src/default_capabilities/plan_service_capability.cpp)

Use the installed AllowedCollisionMatrix, GetPlanningScene, and ApplyPlanningScene definitions under /opt/ros/humble/share/moveit_msgs as the implementation reference, together with the existing moveit_arm_planner.py.
