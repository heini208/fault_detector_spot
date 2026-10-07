# MoveIt collision avoidance implementation plan

Provide an optional environmental occupancy layer for arm planning. Preserve the
existing motion pipeline, endpoint accuracy, trajectory checks, and contact guard.
The RTAB-Map snapshot approach has been removed; native MoveIt sensor input is the
next step. This document describes the current foundation and remaining work.

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
No new native sensor updater is configured in this revision. Enabling the setting
checks whatever occupancy already exists in MoveIt; an empty scene does not
provide environmental avoidance.

## 3. Validate one sensor source — next

Start with one sensor and correct its geometry before combining inputs. For the
rear lidar, verify the declared frame, physical optical/ray origin, point
coordinates, timestamps, and TF chain. Transform both coordinates and frame
consistently; changing only the frame name is insufficient. Reuse calibration
and standard ROS point-cloud transforms. Investigate the existing driver output
first; an application-side adapter may avoid a driver change if calibration and
the necessary transforms are available.

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

## 4. Configure native MoveIt occupancy

Use the installed MoveIt point-cloud or depth-image updater. Start with the
verified source, a practical voxel resolution, bounded sensing range, and modest
update rate. Reuse its robot filtering and free-space ray integration before
adding custom processing. Inspect the active MoveIt launch/configuration and
request authorization before editing a repository outside this project's scope.

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

Keep readiness separate from user preference: show whether usable sensor data
exists before presenting this as live obstacle avoidance. Define stale-data
behavior in this step; do not add a second mapping lifecycle or silently call an
empty scene protected. This is planning-time avoidance, not continuous replanning
around moving obstacles during execution.

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
