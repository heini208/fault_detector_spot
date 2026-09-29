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

Both planar movement commands use `_start_verified_base_movement`. The executor
builds and retains one absolute odometry-frame goal with `_build_absolute_base_goal`.
If post-move verification times out with a fresh pose outside goal tolerance,
`_correct_absolute_base_goal` resubmits that same goal and walking profile. It
never reapplies the original relative offset or resolves a new tag target.

`base.goal_verification.maximum_correction_attempts` defaults to 2: one initial
move plus at most two corrections. Set it to 0 to disable corrections. After a
correction, the largest position/yaw error normalized by its respective tolerance
must improve by at least `minimum_correction_progress_ratio` (default 0.10)
before another correction is allowed. Otherwise the operation fails early.

Stale/missing pose data, failure to settle while already inside tolerance, action
failure/rejection, and cancellation do not cause a retry. Each attempt uses the
existing action and verification timeouts. Cancellation clears the saved goal
and retry state. Existing configured position and yaw tolerances are unchanged.
