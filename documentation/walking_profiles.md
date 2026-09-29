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
