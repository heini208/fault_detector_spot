# Table of Contents

1. [Structure of Each Command](#structure-of-each-command)
2. [Manipulation Commands](#manipulation-commands)
   - [STOW_ARM](#stowarm)
   - [READY_ARM](#readyarm)
   - [TOGGLE_GRIPPER](#togglegripper)
   - [CLOSE_GRIPPER](#closegripper)
   - [MOVE_ARM_TO_TAG](#movearmtotag)
   - [MOVE_ARM_TO_TAG_AND_WAIT](#movearmtotagandwait)
   - [MOVE_ARM_RELATIVE](#movearmrelative)
3. [Navigation & Mobility Commands](#navigation--mobility-commands)
   - [STAND_UP](#standup)
   - [STOP_BASE](#stopbase)
   - [MOVE_TO_WAYPOINT](#movetowaypoint)
   - [MOVE_BASE_TO_TAG](#movebasetotag)
   - [MOVE_BASE_RELATIVE](#movebaserelative)
4. [Mapping & Localization Commands](#mapping--localization-commands)
   - [START_SLAM](#startslam)
   - [START_LOCALIZATION](#startlocalization)
   - [CREATE_MAP](#createmap)
   - [SWAP_MAP](#swapmap)
   - [STOP_MAPPING](#stopmapping)
   - [DELETE_MAP](#deletemap)
5. [Waypoint & Landmark Management Commands](#waypoint--landmark-management-commands)
   - [ADD_CURRENT_POSITION_WAYPOINT](#addcurrentpositionwaypoint)
   - [ADD_TAG_AS_LANDMARK](#addtagaslandmark)
   - [DELETE_WAYPOINT](#deletewaypoint)
   - [DELETE_LANDMARK](#deletelandmark)
6. [System & Utility Commands](#system--utility-commands)
   - [WAIT_TIME](#waittime)
   - [SCAN_ALL_IN_RANGE](#scanallinrange)
   - [EMERGENCY_CANCEL](#emergencycancel)
   - [ESTOP_STATE](#estopstate)

---

# Command Reference

## Structure of Each Command
- **Command Name:** Unique identifier used in the system.
- **Category / Subtree:** Navigation, Manipulation, System, etc.
- **Description:** Short explanation of what the command does.
- **Parameters:** List of inputs with types, default values, and allowed ranges.
- **Preconditions / Dependencies:** Conditions that must be met before executing the command.
- **Effects / Outcomes:** What the command triggers in the robot or system.
- **Feedback / Status Updates:** How the system reports execution progress or completion.
- **Errors / Warnings:** Known failure cases or safety considerations.
- **Example Usage:** Optional example of how to issue the command.

## Manipulation Commands

### STOW_ARM
- **Category:** Manipulation
- **Description:** Retracts the Spot arm into its stowed position on the robot's back.
- **Parameters:** None.
- **Preconditions:** Arm should be powered.
- **Effects:** The arm moves to the stow configuration.
- **Feedback:** Returns `RUNNING` during movement, `SUCCESS` when stowed.
- **Errors:** Returns `FAILURE` if the `RobotCommand` action fails or is rejected.

### READY_ARM
- **Category:** Manipulation
- **Description:** Deploys the Spot arm into a "ready" position in front of the robot. **Note:** This position extends fairly far in front of the robot. It is not necessary for other manipulation commands, as they can typically execute directly from a stowed state.
- **Parameters:** None.
- **Preconditions:** Arm should be powered.
- **Effects:** The arm moves to the ready configuration.
- **Feedback:** Returns `RUNNING` during movement, `SUCCESS` when position is reached.
- **Errors / Warnings:** Returns `FAILURE` if the `RobotCommand` action fails. **Warning:** Because the arm extends significantly forward, the command range and reach should be verified experimentally first to avoid accidental collisions with the environment.

### TOGGLE_GRIPPER
- **Category:** Manipulation
- **Description:** Toggles the gripper state between open and closed.
- **Parameters:** None.
- **Preconditions:** Arm must be deployed.
- **Effects:** If open, the gripper closes; if closed, it opens.
- **Feedback:** Returns `SUCCESS` upon completion.
- **Errors:** Returns `FAILURE` if the command fails.

### CLOSE_GRIPPER
- **Category:** Manipulation
- **Description:** Fully closes the gripper.
- **Parameters:** None.
- **Preconditions:** Arm must be deployed.
- **Effects:** The gripper closes (0.0 fraction open).
- **Feedback:** Returns `RUNNING` during actuation, `SUCCESS` upon completion.
- **Errors:** Returns `FAILURE` if the command fails.

### MOVE_ARM_TO_TAG
- **Category:** Manipulation
- **Description:** Moves the gripper to a specific fiducial tag (AprilTag) currently visible to the robot. The robot calculates the transform to the tag and executes the arm movement.
- **Parameters:**
  - `tag_id` (int): The ID of the AprilTag to reach.
  - `offset` (Pose, optional): An offset relative to the tag frame (e.g., `{x: -0.2}` to stop 20cm in front of it). **CRITICAL:** Without a defined offset, the robot will attempt to move the gripper's TCP (Tool Center Point) exactly to the tag's origin, causing the gripper to collide with the tag/surface.
- **Preconditions:** The specific `tag_id` must be currently visible and reachable (`reachable_tags`, green in the test-UI).
- **Effects:** The arm moves so the hand aligns with the target tag's pose (plus offset).
- **Feedback:** "Found goal tag {id}", "Waiting for TF".
- **Errors:** Returns `FAILURE` if the tag is not visible or TF lookup times out.

### MOVE_ARM_TO_TAG_AND_WAIT
- **Category:** Manipulation
- **Description:** Executes `MOVE_ARM_TO_TAG` and then waits for a specified duration, usually to allow for gripping or inspection.
- **Parameters:**
  - `tag_id` (int): The ID of the AprilTag to reach.
  - `duration` (float): Time to wait after reaching the tag.
  - `offset` (Pose, optional): An offset relative to the tag frame (e.g., `{x: -0.2}` to stop 20cm in front of it). **CRITICAL:** Without a defined offset, the robot will attempt to move the gripper's TCP (Tool Center Point) exactly to the tag's origin, causing the gripper to collide with the tag/surface.
- **Preconditions:** The specific `tag_id` must be currently visible and reachable (`reachable_tags`, green in the test-UI).
- **Effects:** Moves arm to tag, then pauses execution.
- **Feedback:** Same as `MOVE_ARM_TO_TAG` plus wait status.
- **Errors:** Returns `FAILURE` if move fails.

### MOVE_ARM_RELATIVE
- **Category:** Manipulation
- **Description:** Moves the arm by a relative offset (X, Y, Z) specified in the robot's body frame, maintaining the current orientation.
- **Parameters:**
  - `x`, `y`, `z` (float): The relative distance to move in meters.
- **Preconditions:** Arm must be deployed; TF transform to hand must be valid.
- **Effects:** The hand position shifts by the specified delta.
- **Feedback:** "Waiting for TF -> hand".
- **Errors:** Returns `FAILURE` if TF lookup times out.

---

## Navigation & Mobility Commands

### STAND_UP
- **Category:** Mobility
- **Description:** Commands the Spot robot to stand up on all four legs.
- **Parameters:** None.
- **Preconditions:** Robot motors must be powered.
- **Effects:** Robot transitions to a standing posture.
- **Feedback:** `RUNNING` during transition, `SUCCESS` when standing.
- **Errors:** Fails if the robot cannot stand or is obstructed.

### STOP_BASE
- **Category:** Navigation
- **Description:** Immediately stops the robot base by publishing zero velocity commands.
- **Parameters:** None.
- **Preconditions:** None.
- **Effects:** `cmd_vel` is set to linear: 0.0, angular: 0.0.
- **Feedback:** Always returns `SUCCESS`.
- **Errors:** None.

### MOVE_TO_WAYPOINT
- **Category:** Navigation
- **Description:** Retrieves a saved waypoint pose from the map file and navigates the robot to it using Nav2.
- **Parameters:**
  - `map_name` (string): The name of the map file.
  - `waypoint_name` (string): The identifier of the stored waypoint.
- **Preconditions:** Map JSON must exist; Nav2 must be active (Localization mode).
- **Effects:** The robot navigates to the waypoint coordinates.
- **Feedback:** "Navigating...", "Navigation succeeded/failed".
- **Errors:** Returns `FAILURE` if waypoint/map missing or navigation fails.

### MOVE_BASE_TO_TAG
- **Category:** Navigation
- **Description:** Navigates the robot base to a specific pose relative to a visible AprilTag.
- **Parameters:**
  - `tag_id` (int): The target tag ID.
  - `offset` (Pose): Desired pose relative to the tag frame. **Important:** If the tag is not on the ground (e.g., on a wall), the offset is critical to define a valid ground-plane target. Without a proper offset, the action may fail to conclude as the robot cannot drive into the wall/air, although collision avoidance will typically prevent impact.
- **Preconditions:** Tag must be visible to the robot cameras.
- **Effects:** Robot drives to position itself relative to the tag.
- **Obstacle Avoidance:** If an obstacle (e.g., a person) enters the robot's path or the target zone, the robot will actively avoid collision and subsequently attempt to return to the target path.
  - **Note:** If this automatic recovery is not desired (i.e., you want the robot to stop immediately upon obstruction), use the `ESTOP_STATE` command to disable motion.
- **Feedback:** "Moving base to tag {id}".
- **Errors:** Fails if tag is not visible.

### MOVE_BASE_RELATIVE
- **Category:** Navigation
- **Description:** Moves the robot base by a specified relative distance from its current position.
- **Parameters:**
  - `x` (float): Forward/Backward distance (meters).
  - `y` (float): Left/Right distance (meters).
  - `yaw` (float): Rotation (radians).
- **Preconditions:** Nav/Locomotion system active.
- **Effects:** Robot moves by the delta specified.
- **Obstacle Avoidance:** If an obstacle (e.g., a person) enters the robot's path or the target zone, the robot will actively avoid collision and subsequently attempt to return to the target path.
  - **Note:** If this automatic recovery is not desired (i.e., you want the robot to stop immediately upon obstruction), use the `ESTOP_STATE` command to disable motion.
- **Feedback:** "Executing relative base move".
- **Errors:** Fails if path is blocked.

---

## Mapping & Localization Commands

### START_SLAM
- **Category:** Mapping
- **Description:** Starts the SLAM (Simultaneous Localization and Mapping) process using an existing map to extend it.
- **Parameters:**
  - `active_map_name` (implicit): Reads active map from blackboard.
- **Preconditions:** An active map must be selected.
- **Effects:** Launches SLAM Toolbox in mapping mode.
- **Feedback:** "Mapping enabled (SLAM running)".
- **Errors:** Fails if no active map is set.

### START_LOCALIZATION
- **Category:** Mapping
- **Description:** Starts the localization process using a static map (no map updates).
- **Parameters:**
  - `active_map_name` (implicit): Reads active map from blackboard.
- **Preconditions:** An active map must be selected.
- **Effects:** Launches SLAM Toolbox/Nav2 in localization mode.
- **Feedback:** "Launching Localization".
- **Errors:** Fails if no active map is set.

### CREATE_MAP
- **Category:** Mapping
- **Description:** Initializes a new mapping session, deleting any existing data for the specified name, and starts SLAM.
- **Parameters:**
  - `map_name` (string): The name for the new map.
- **Preconditions:** None.
- **Effects:** Creates empty map files and starts mapping.
- **Feedback:** "No map_name provided" if missing.
- **Errors:** Fails if parameters are invalid.

### SWAP_MAP
- **Category:** Mapping
- **Description:** Switches the active map context. If SLAM is running, restarts it with the new map.
- **Parameters:**
  - `map_name` (string): The name of the map to load.
- **Preconditions:** Map files must exist.
- **Effects:** Updates `active_map_name`, restarts SLAM/Localization.
- **Feedback:** "Changed to map '{name}'".
- **Errors:** Fails if `map_name` is missing.

### STOP_MAPPING
- **Category:** Mapping
- **Description:** Stops the currently running SLAM/Localization process and saves the map state.
- **Parameters:** None.
- **Preconditions:** SLAM or Nav2 must be running.
- **Effects:** Serializes map, saves files, kills processes.
- **Feedback:** "Stop requested, waiting for termination".
- **Errors:** None.

### DELETE_MAP
- **Category:** Mapping
- **Description:** Permanently deletes files associated with a map.
- **Parameters:**
  - `map_name` (string): Map to delete.
- **Preconditions:** None.
- **Effects:** Removes .db, .posegraph, .json files.
- **Feedback:** "Deleted map files for: {name}".
- **Errors:** None.

---

## Waypoint & Landmark Management Commands

### ADD_CURRENT_POSITION_WAYPOINT
- **Category:** Database
- **Description:** Saves current estimated pose as a named waypoint.
- **Parameters:**
  - `waypoint_name` (string): Name for the location.
  - `map_name` (string): Map to associate with.
- **Preconditions:** Valid pose estimation available.
- **Effects:** Updates JSON registry.
- **Feedback:** "Saved waypoint '{name}'".
- **Errors:** Fails if localization not running.

### ADD_TAG_AS_LANDMARK
- **Category:** Database
- **Description:** Saves the pose of a visible AprilTag as a static landmark.
- **Parameters:**
  - `tag_id` (int): Visible tag ID.
  - `landmark_name` (string): Name for the landmark.
  - `map_name` (string): Map to associate with.
- **Preconditions:** Tag must be visible in map frame.
- **Effects:** Transforms tag pose to map frame and saves to JSON.
- **Feedback:** "Saved landmark '{name}'".
- **Errors:** Fails if tag not visible or TF unavailable.

### DELETE_WAYPOINT
- **Category:** Database
- **Description:** Removes a specific waypoint from the registry.
- **Parameters:**
  - `waypoint_name` (string): Waypoint to remove.
  - `map_name` (string): Associated map.
- **Preconditions:** Waypoint must exist.
- **Effects:** Updates JSON file.
- **Feedback:** "Deleted waypoint '{name}'".
- **Errors:** Fails if not found.

### DELETE_LANDMARK
- **Category:** Database
- **Description:** Removes a specific landmark from the registry.
- **Parameters:**
  - `landmark_name` (string): Landmark to remove.
  - `map_name` (string): Associated map.
- **Preconditions:** Landmark must exist.
- **Effects:** Updates JSON file.
- **Feedback:** "Deleted landmark '{name}'".
- **Errors:** Fails if not found.

---

## System & Utility Commands

### WAIT_TIME
- **Category:** Utility
- **Description:** Pauses execution for a defined period of time.
- **Parameters:**
  - `duration` (float): Time to wait in seconds.
- **Preconditions:** None.
- **Effects:** Delays the behavior tree execution.
- **Feedback:** "Waiting...".
- **Errors:** None.

### SCAN_ALL_IN_RANGE
- **Category:** Utility
- **Description:** Performs a scanning routine (e.g., looking around with cameras) to detect all tags or objects in the immediate vicinity.
- **Parameters:** None.
- **Preconditions:** Cameras/Sensors active.
- **Effects:** Updates the `visible_tags` or object list on the blackboard.
- **Feedback:** "Scanning...".
- **Errors:** Fails if sensors unavailable.

### EMERGENCY_CANCEL
- **Category:** System
- **Description:** Triggers a high-priority cancellation of all active movements and behaviors.
- **Parameters:** None.
- **Preconditions:** None.
- **Effects:** Stops base, arms, and resets behavior tree state.
- **Feedback:** "Emergency Cancel Triggered".
- **Errors:** None.

### ESTOP_STATE
- **Category:** System
- **Description:** Toggles or checks the software E-Stop state of the robot.
- **Parameters:**
  - `state` (string/bool): "SET" or "RELEASE".
- **Preconditions:** None.
- **Effects:** Cuts motor power or allows power depending on state.
- **Feedback:** "E-Stop Set" or "E-Stop Released".
- **Errors:** None.