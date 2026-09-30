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

Relative and tag-relative base commands are resolved into a
`BaseMovementPlan`. All direct planar Spot movements, including corrections,
pass through `BaseMovementExecutor._submit_movement_plan`. Trajectory
construction belongs to `BaseMotionPlanner`; the behavior tree only starts the
executor and polls its typed result.

After the driver reports success, `BaseGoalVerifier` measures the odom-frame
base pose and determines physical settling independently from target accuracy.
Driver success can precede physical arrival. Measurable progress toward the
target renews the five-second verification window, allowing slower precision
walking to finish. Verification remains bounded by the executor's result
timeout (30 seconds by default, measured from driver success). Missing poses,
noise, and motion away from the target do not renew the window.
`BaseCorrectionPolicy` then makes only the bounded `CORRECT` or `FAIL`
decision. It does not decide how a target is rebuilt.

Relative movement uses a frozen-target strategy. If correction is allowed, the
same resolved absolute `BaseMovementPlan` is submitted again. The original
relative offset is not reapplied.

Tag-relative movement uses a fresh-target strategy. Once the base is physically
settled, execution enters `WAITING_FOR_FRESH_TAG`. Only tag observations newer
than that settle timestamp may be used. Several unique observations must remain
within the configured position and yaw spans before they are accepted as
stable. Observations are transformed into odom at their capture timestamps
before comparison, so body sway does not appear as tag motion. Missing
capture-time transforms are retried within the observation timeout.
When the offset frame is the selected `Tag_N`, its offset and heading use
that same observation. Zero yaw follows the horizontal projection of the
tag's inward (-Z) normal; it does not mean zero odom yaw. The observation
yaw-span limit is 3 degrees, independently of the endpoint heading tolerance.
The original semantic tag request is then resolved again from that
stable observation. If the measured base pose is already within tolerance of
the fresh target, the operation succeeds; otherwise the freshly resolved plan
is used for the next correction.

`base.correction.maximum_attempts` defaults to 2, meaning one initial movement
plus at most two corrections. Set it to 0 to disable corrections. After a
correction, normalized position/yaw error must improve by at least
`base.correction.minimum_progress_ratio` (default 0.10) before another
correction is allowed.

The post-settle tag window is bounded by
`base.tag_correction.observation_timeout_sec`. Stability is configured with
`base.tag_correction.stability.required_samples`,
`maximum_position_span_m`, `maximum_yaw_span_rad`, and
`maximum_sample_span_sec`. These values should be tuned from real robot
measurements. The default sample span is 4 seconds to accommodate three
distinct observations at the observed roughly 1 Hz Spot world-object cadence.
Repeated publications of the same observation do not count as new samples.
The 5-second observation timeout starts when physical settling is confirmed.

Cancellation retains base-operation ownership until the active RobotCommand
reaches a terminal state. Goal-response and result timeouts follow the same
ownership rule. Emergency stop remains an independent preemption path and does
not wait for normal cancellation bookkeeping.
