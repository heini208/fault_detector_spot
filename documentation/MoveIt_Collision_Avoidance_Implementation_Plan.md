# MoveIt collision avoidance implementation plan

Add optional environmental collision checking to the existing arm planner. Its purpose is to reduce avoidable collisions and failed movements before execution. The existing collision guard continues to react during movement. This is a best-effort planning aid, not a replacement safety system.

Keep the current motion generator, endpoint tolerances, calibration, speed settings, trajectory validation, and execution path. When this layer is disabled, sensor obstacles must not affect planning. When enabled, they may change the route or prevent a plan, but must not change the requested target or relax accuracy requirements.

This simplified plan supersedes the earlier custom-capability, private-snapshot, and new-action proposal. It reflects the clarified requirements of 7 October 2026.

## Current pilot

Steps 1, 2, and 4 are implemented, together with the sensor integration in
Step 3. The feature defaults to disabled. The default source remains
`/depth_registered/frontleft/points`; `environment_collision_lidar:=true`
also selects the rear lidar. Additional cameras remain deferred.
The lidar integration uses existing ROS components and saved SDK calibration;
it requires no change to the Spot driver or sibling `spot_moveit_config` package.

Initial pilot validation: 444 relevant offline tests passed across 25 modules.
After simplifying checkbox consumption, 81 focused UI tests passed and the
copied application overlay was rebuilt. Both project
packages build in the isolated overlay; launch parameters normalize successfully,
and the matching perception plugin loads. No live updater, application restart,
or robot motion was started during implementation. Hardware validation remains
pending. The subsequent lidar change passed 78 focused offline tests, including
SDK calibration extraction, actual Humble component parameter loading, selected
sensor freshness, and collision-policy/planner checks. Read-only SDK inspection
verified the physical mount transform on the attached robot. The new lidar
components have not been started or validated live.

Launch-time `environment_collision_avoidance:=true` loads the standard occupancy
updater into MoveIt and enables the planner's observation gate. Checked requests
wait for fresh stationary odometry, clear the map, require a usable filtered
cloud from each selected updater acquired after clear acknowledgment, and confirm a nonempty map in `body`
before applying collision policy and planning. Every checked segment refreshes
the map; this first pilot has no reuse cache. Movement or stale observations
during planning discard the result. The base must also remain stationary during
execution; this feature does not add continuous obstacle replanning.

The one-shot checkbox is in the basic arm controls. Explicit bypass skips map
refresh and sensor availability checks while still applying the occupancy-only
collision exception. Contact/custom-final motions and contact backoff receive
their appropriate bypass explicitly. Self-collision checks and existing guards
remain active. The launch option is a session setting, not a live parameter
switch; disabling it requires a fresh application/MoveIt launch so an old
occupancy map cannot remain in a server used by the original planning path.
The checkbox clears immediately when submitting a basic arm movement, including
when the server later rejects it or is unavailable. Check it again manually to
bypass another submission. Server feedback cannot re-arm or change this UI choice.

The new ROS boolean requires rebuilding **both `fault_detector_msgs` and
`fault_detector_spot`**, then running all application processes from the same
overlay. Do not mix old generated interfaces with the updated Python code.
The standard `moveit_ros_perception` point-cloud plugin is also required when
enabling the pilot; the originally inspected Humble installation lacks it.
For this development session, an isolated build of the official 2.5.9 point-cloud
plugin and isolated builds of both project packages are available under `/tmp`.
No system package or shared workspace installation was changed. The perception
build disables unused OpenGL components; source algorithms are unchanged.

The prepared environment can be loaded in a fresh terminal with:

```bash
source /opt/ros/humble/setup.bash
source /home/marcel/Projects/spot/spot_ws/install/setup.bash
source /tmp/spot_collision_build/install/setup.bash
source /tmp/spot_collision_perception/install/moveit_ros_perception/share/moveit_ros_perception/local_setup.bash
```

Those temporary overlays are test artifacts for this session, not a permanent
deployment. Their build notes are in `/tmp/spot_collision_perception/BUILD_NOTES.md`.

For the next hardware session, load the environment above in each terminal:

1. Keep the driver running and close any previous fault-detector/MoveIt instance.
   Start the updated application with:

   ```bash
   ros2 launch fault_detector_spot fault_detector_launch.py environment_collision_avoidance:=true
   ```

   Use the normal preparation workflow to put the arm in its working posture
   with clear space around it. The planning diagnostic does not ready the arm.
   Then keep the base stationary and application arm commands idle.

2. Inspect the map using the existing RViz configuration:

   ```bash
   rviz2 -d /home/marcel/Projects/spot/spot_ws/src/spot_moveit_config/config/moveit.rviz
   ```

   Its fixed frame is `body` and planning scene topic is
   `/monitored_planning_scene`. Use RViz only to view the scene during these
   diagnostic runs. A test object in the front-left depth camera's view should
   appear at the correct position, and the robot should be filtered. The sensor
   mount/probe geometry is not modeled yet. Resolve obvious misalignment or
   self-occupancy before testing movement; one camera also leaves blind spots.

3. Choose a previously reachable nearby **hand pose in body**, including its
   quaternion. Do not use probe-tip coordinates or assume identity orientation.
   Use the same target throughout the comparison. This command is a template:
   replace `X Y Z QX QY QZ QW` with the chosen pose.

   ```bash
   cd /home/marcel/Projects/spot/spot_ws/src/fault_detector_spot
   goal=(--target X Y Z --quaternion QX QY QZ QW)
   python3 scripts/check_moveit_environment.py "${goal[@]}" --cartesian
   ```

   The clear-workspace baseline must report `success` with trajectory points.
   The script plans only and sends no robot execution command. Missing/stale
   observations or an invalid start/goal are prerequisite failures, not evidence
   of collision avoidance. Do not continue the comparison until the baseline works.

4. Place a visible obstacle across that straight planned path, away from the
   stationary robot, and confirm it appears in the map. Run these sequentially:

   ```bash
   python3 scripts/check_moveit_environment.py "${goal[@]}" --cartesian
   python3 scripts/check_moveit_environment.py "${goal[@]}" --cartesian --ignore-environment-collisions
   python3 scripts/check_moveit_environment.py "${goal[@]}" --cartesian
   ```

   Expected: blocked/incomplete, success, then blocked/incomplete again, with
   a fresh map for each checked attempt. The identical target must pass with
   bypass for this to demonstrate the occupancy toggle. The script changes the
   shared map/ACM and leaves its final policy in place, so keep UI/RViz planning
   idle. After a timeout or interruption, establish that server work finished
   before another request. These commands never execute the blocked trajectory.
   Normal planning can be compared by omitting `--cartesian`; it may route around
   the obstacle instead of failing. Output reports status and point count, not a
   saved or automatically animated trajectory.

5. Remove the obstacle. After the planning checks pass, use a familiar small basic
   movement in a clear workspace to test the UI checkbox. It must clear immediately
   on submission, and the following unchecked command must use checking again.
   Check it again manually after any rejected submission. Unrelated base, gripper,
   wait, and saved-workflow commands must not consume the checkbox.

6. Test absent-source handling separately afterward. Missing/stale input must
   prevent checked planning; explicit bypass can still request an ordinary plan.
   Contact workflow and endpoint accuracy checks follow these initial checks.

These are pending hardware checks, not claims of demonstrated avoidance.

### Optional lidar input without a driver change

The existing `/velodyne/points` stream stays unchanged. Its coordinates are at
the odometry origin; the driver's ROS output omits the physical sensor transform
that is present in the SDK response. `config/moveit_lidar_calibration.yaml`
contains the attached robot's SDK `body → sensor` calibration captured on
7 October 2026. Refresh this file when the lidar mount changes or using another
robot; it is not a universal Spot mounting pose.

The application launch starts one optional component container containing the
standard `tf2_ros::StaticTransformBroadcasterNode` and
`rtabmap_util::PointCloudAssembler`. The latter uses `max_clouds=1` to transform
each existing cloud into `moveit_lidar_sensor` at its acquisition timestamp,
without accumulating scans. It publishes `/moveit_environment/lidar/points`.
MoveIt's second standard updater consumes this topic and publishes
`/moveit_environment/lidar/filtered_cloud`. Mapping's arm-box filter is not used
on this branch; MoveIt uses its robot model for self-filtering.

After building and sourcing the application overlay, keep the existing driver
running and launch the application with:

```bash
ros2 launch fault_detector_spot fault_detector_launch.py environment_collision_avoidance:=true environment_collision_lidar:=true
```

In RViz with fixed frame `body`, compare `/velodyne/points` and
`/moveit_environment/lidar/points`: the surfaces must overlap even though their
coordinate frames differ. Check that `moveit_lidar_sensor` is at the physical
lidar, then inspect the filtered lidar cloud and planning scene. Place an object
within two metres of the lidar on a side outside the front-left camera's view;
confirm useful additional coverage and check for robot self-occupancy before
testing movement. Run the planning-only comparisons above with `--lidar` added
to **every** diagnostic command, so both selected sensors are required.

Lidar is opt-in: leave the new launch argument false when it is absent.
If selected but unavailable, checked planning waits and fails rather than
silently proceeding with camera-only coverage. Explicit obstacle bypass retains
its existing behavior. No front-right or hand camera is added in this step.

To refresh calibration directly from the SDK, using your existing connection
configuration and no driver access:

```bash
python3 scripts/read_spot_lidar_calibration.py --robot-config config/spot_ros_lan.yaml --output /tmp/spot_lidar_calibration.yaml
```

Pass `environment_lidar_calibration:=/tmp/spot_lidar_calibration.yaml` at launch
to use that export. This setup command only authenticates and reads metadata;
it takes no lease and sends no movement commands. The running cloud components
use ROS data and the saved calibration, with no additional SDK polling.

Adding lidar improves coverage; it does not introduce immediate occupancy decay.
Old obstacles in an idle RViz view still depend on OctoMap free-space evidence.
Each checked planning request clears and rebuilds the local map as described
above. The reported moving-wall delay has not yet been measured conclusively.

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
keeps the direct planning path and original cancellation behavior when disabled.
Command/UI wiring and the initial camera source are now connected as described
in Current pilot. Offline tests cover effective
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

Implemented through semantic commands, ROS messages/adapters, recording,
workflow factories, Behavior Tree expansion, and executor continuations. Offline
tests cover policy preservation through corrections, retries, and backtracking.

### Step 3 Connect one sensor and local map refresh

Configure the standard point-cloud occupancy updater in the existing MoveIt launch.
The pilot uses the front-left depth cloud and optional lidar with the
SDK-calibrated origin described above. For **/velodyne/points**, branch before the mapping
arm-box filter, which can also remove actual objects near the arm. Keep the mapping
branch unchanged; use MoveIt's robot self-filter for planning.

Mapping need not run, but the lidar driver must supply usable data. Verify topic, frame, timestamps, QoS, and self-filter behavior. Do not start mapping just to obtain a planning cloud.

The current lidar cloud is expressed at the odometry origin. MoveIt's standard
updater uses the cloud frame origin for free-space rays, so a valid TF alone is
insufficient. Before using this topic, obtain the actual calibrated acquisition
origin and transform the point coordinates into that sensor frame at the cloud
timestamp; changing only `frame_id` is incorrect. Reuse existing TF/calibration
and leave the mapping stream unchanged. The SDK metadata export and standard
ROS components now provide this conversion without a driver change. If the
physical origin is unavailable, use the camera-only option. See the
[versioned updater implementation](https://github.com/moveit/moveit2/blob/2.5.9/moveit_ros/perception/pointcloud_octomap_updater/src/pointcloud_octomap_updater.cpp).

Implement the bounded refresh described above. Choose a modest local range and voxel resolution for avoiding obvious obstacles. Tune for useful coverage and low overhead rather than millimetre surface reconstruction. Do not increase self-filter padding until nearby real obstacles disappear.

Do not change target tolerances or movement speeds to compensate for noisy occupancy. Fix the input/filtering or disable the optional layer for that motion.

**Done when:** nearby obstacles appear usefully, the arm is filtered, and sensor unavailability is reported clearly. Bypass behavior still matches the baseline.

### Step 4 Add the one-shot basic movement UI option

Add the checkbox to the basic arm controls and include its value in the submitted
intent. Clear it before sending so a second click cannot reuse it. A rejected or
failed submission leaves it cleared; the user checks it again for another override.

Unrelated base, gripper, wait-only, or saved-workflow commands do not consume it. Display whether the active command is checking environmental obstacles. Avoid confirmation dialogs or a separate global UI state machine.

Changing the checkbox while a movement is active applies to a future command, not the current trajectory.

**Done when:** exactly the intended next basic movement receives the override and later movements use their own settings.

Implemented as a local checkbox read and clear at submission. Admission signals,
pending override records, and revision tracking were removed. The command's
captured boolean still travels through the normal application command path.

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

Sensor parameters are supplied by a scoped parameter file around the existing
MoveIt launch include in fault_detector_spot. No sibling package edits or new
production package are needed.

Leave out custom planning actions, private scene cloning, per-plan scene epochs, new C++ capabilities, floating-base model changes, alternate planners, automatic replanning, continuous execution monitoring, formal coverage certification, and GPU mapping. Revisit an individual item only if a concrete limitation requires it.

## Why this simpler approach fits

The earlier plan optimized for concurrent clients, strong scene consistency, and future reactive avoidance. Those exceed the clarified requirement. A shared policy set before each plan is a reasonable compromise for the existing serialized application, provided the single-client assumption and outstanding-request handling are enforced.

Keeping the same planner, Cartesian service, robot model, and executor minimizes changes that could affect accuracy and reliability. The disposable local map is only an additional planning input. It can cause false rejections or extra delay when enabled; the toggle explicitly restores baseline planning behavior.

MoveIt's standard perception and scene services provide the necessary mechanisms. [Perception overview](https://moveit.picknik.ai/humble/doc/examples/perception_pipeline/perception_pipeline_tutorial.html), [versioned planning service](https://github.com/moveit/moveit2/blob/2.5.9/moveit_ros/move_group/src/default_capabilities/plan_service_capability.cpp)

Use the installed AllowedCollisionMatrix, GetPlanningScene, and ApplyPlanningScene definitions under /opt/ros/humble/share/moveit_msgs as the implementation reference, together with the existing moveit_arm_planner.py.
