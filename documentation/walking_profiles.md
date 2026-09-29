# Testing walking profiles from the UI

The base movement controls have a **Walking profile** dropdown, defaulting to
**Normal**. Select **Precision** before **Move to Tag** or **Move Base by Offset**.
The selection is copied into each submitted command. Changing the dropdown does
not alter queued or active commands, waypoint navigation, or stand/sit commands.

Per-command selection overrides the startup selectors in `config/base_motion.yaml`.
Commands without a selection still use those configured defaults. Recordings
preserve the selection; older recordings without the field remain readable.
Precision is an experimental slower profile, not an accuracy guarantee.

This adds `walking_profile` to the `OperationalIntent` and `CommandPayload`
ROS messages. Rebuild `fault_detector_msgs` and its consumers together before
using the updated UI. The isolated test build does not update the live install.

## Automatic endpoint correction

Relative and tag-relative base commands are first resolved into a
`BaseMovementPlan`. Initial execution and correction attempts both pass through
`BaseMovementExecutor._submit_movement_plan`, which is the common submission
boundary for direct planar Spot movement.

After the driver reports success, `BaseGoalVerifier` checks the measured
odom-frame endpoint and settling behavior. `BaseCorrectionPolicy` independently
decides whether an inaccurate endpoint should retry the frozen movement plan.

`base.correction.maximum_attempts` defaults to 2: one initial move plus at most
two corrections. Set it to 0 to disable corrections. After a correction, the
largest position/yaw error normalized by its respective goal tolerance must
improve by at least `base.correction.minimum_progress_ratio` (default 0.10)
before another correction is allowed. Otherwise the operation fails early.

A frozen retry rebuilds the Spot command from the same resolved
`BaseMovementPlan`. It does not reapply the original relative offset or resolve
a fresh tag target. Stale or missing measured pose data, failure to settle while
already inside tolerance, RobotCommand failure or rejection, and cancellation do
not cause a retry. Each attempt uses the existing action and verification
timeouts. Existing configured position and yaw tolerances are unchanged.
