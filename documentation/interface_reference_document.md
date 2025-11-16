# Command & Interface Reference

This document provides a detailed reference of the ROS 2 interfaces used by the Fault Detector Spot system, with a focus on how to:

- Publish commands (from UIs, tests, or agents)
- Subscribe to feedback and status
- Work with mapping, navigation, and recording interfaces

It is intended for developers implementing **custom user interfaces** or **automated control agents** on top of the system.

---

## 1. Command Model

### 1.1 BasicCommand and ComplexCommand

Commands are sent as `fault_detector_msgs/ComplexCommand` messages on a single topic:

- **Topic:** `fault_detector/commands/complex_command`
- **Type:** `fault_detector_msgs/ComplexCommand`
- **Direction:** UI / external → Behaviour Tree

`ComplexCommand` wraps a `BasicCommand` and optional context:

- `command` (`fault_detector_msgs/BasicCommand`)
    - `header.stamp` – time when the command was issued
    - `command_id` – one of the `CommandID` enum values (from `command_ids.py`)
- Optional fields (used depending on `command_id`):
    - `tag` (`fault_detector_msgs/TagElement`)
        - `id` – AprilTag ID
        - `pose` – `geometry_msgs/PoseStamped` (tag pose)
    - `offset` (`geometry_msgs/PoseStamped`)
        - `header.frame_id` – one of the `FrameNames` enum (`BODY`, `HAND`, `MAP_FRAME`, …)
        - `pose.position.{x,y,z}` – positional offset
        - `pose.orientation` – `geometry_msgs/Quaternion`
    - `map_name` (`string`) – map identifier
    - `waypoint_name` (`string`) – waypoint or landmark name
    - `wait_time` (`float`) – dwell or scanning duration in seconds
    - `orientation_mode` (`string`) – one of the `OrientationModes` values (for manipulation commands)

The UI builds these messages via helper methods such as:

- `Fault_Detector_UI.build_basic_command(command_id)`
- `ManipulationControls.handle_full_message(...)`
- `BaseMovementControls.build_move_base_command(...)`
- `NavigationControls.handle_add_waypoint()`, etc.

### 1.2 Command IDs

Command IDs are defined in `fault_detector_spot.behaviour_tree.commands.command_ids.CommandID`.  
They map directly to behaviour-tree subtrees.

Examples (non-exhaustive):

- Manipulation:
    - `MOVE_ARM_TO_TAG`
    - `MOVE_ARM_RELATIVE`
    - `MOVE_ARM_TO_TAG_AND_WAIT`
    - `READY_ARM`
    - `STOW_ARM`
    - `TOGGLE_GRIPPER`
    - `CLOSE_GRIPPER`
    - `SCAN_ALL_IN_RANGE`
- Base:
    - `MOVE_BASE_TO_TAG`
    - `MOVE_BASE_RELATIVE`
    - `STAND_UP`
    - `STOP_BASE`
- Mapping & Navigation:
    - `CREATE_MAP`
    - `DELETE_MAP`
    - `SWAP_MAP`
    - `START_SLAM`
    - `START_LOCALIZATION`
    - `STOP_MAPPING`
    - `ADD_CURRENT_POSITION_WAYPOINT`
    - `DELETE_WAYPOINT`
    - `MOVE_TO_WAYPOINT`
    - `ADD_TAG_AS_LANDMARK`
    - `DELETE_LANDMARK`
- System:
    - `EMERGENCY_CANCEL`
    - `ESTOP_STATE`
    - `WAIT_TIME`

Refer to the [System Design](System_Design.md) Document’s **Available Commands** table and the [code](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fcommands%2Fcommand_ids.py) for the precise set and semantics.

---

## 2. Command Input Interfaces

### 2.1 Command Topic

- **Topic:** `fault_detector/commands/complex_command`
- **Type:** `fault_detector_msgs/ComplexCommand`
- **Direction:** External → Behaviour Tree

Any UI or agent can publish here. The Behaviour Tree’s `CommandSubscriber` receives these messages, converts them into internal command objects, and appends them to the command buffer on the blackboard.

#### 2.1.1 Example: Simple command (no extra context)

To toggle the gripper:

```python
from fault_detector_msgs.msg import ComplexCommand, BasicCommand
from std_msgs.msg import Header
from fault_detector_spot.behaviour_tree.commands.command_ids import CommandID

cmd = ComplexCommand()
basic = BasicCommand()
basic.header = Header()
basic.header.stamp = node.get_clock().now().to_msg()
basic.command_id = CommandID.TOGGLE_GRIPPER
cmd.command = basic
publisher.publish(cmd)
```

#### 2.1.2 Example: Move arm relative to a tag

1. Set `command_id = MOVE_ARM_RELATIVE`.
2. Set `tag` to the desired visible tag.
3. Set `offset` with position/orientation and frame.

```python
from fault_detector_msgs.msg import ComplexCommand, TagElement
from geometry_msgs.msg import PoseStamped, Quaternion
from fault_detector_spot.behaviour_tree.commands.command_ids import CommandID, FrameNames

cmd = ComplexCommand()
cmd.command = basic_move = BasicCommand()
basic_move.header.stamp = node.get_clock().now().to_msg()
basic_move.command_id = CommandID.MOVE_ARM_RELATIVE

# Tag info (for example, from /fault_detector/state/visible_tags)
tag_element = TagElement()
tag_element.id = 23
tag_element.pose = some_visible_tag_pose  # PoseStamped
cmd.tag = tag_element

# Offset: 10 cm in front, slightly up
cmd.offset = PoseStamped()
cmd.offset.header.frame_id = FrameNames.HAND.value  # or MAP_FRAME, BODY, etc.
cmd.offset.pose.position.x = 0.10
cmd.offset.pose.position.y = 0.0
cmd.offset.pose.position.z = 0.05
cmd.offset.pose.orientation = Quaternion(w=1.0)  # identity orientation

publisher.publish(cmd)
```

The provided Qt-based UI uses these same fields (see [manipulation_controls.py](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fui_classes%2Fmanipulation_controls.py) and [base_movement_controls.py](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fui_classes%2Fbase_movement_controls.py) ).

---

## 3. State and Feedback Interfaces

The Behaviour Tree publishes multiple topics that UIs and monitors can subscribe to.

### 3.1 Visible and Reachable Tags

- **Visible tags**
    - Topic: `fault_detector/state/visible_tags`
    - Type: `fault_detector_msgs/TagElementArray`
    - Direction: BT → UI / external

- **Reachable tags**
    - Topic: `fault_detector/state/reachable_tags`
    - Type: `fault_detector_msgs/TagElementArray`
    - Direction: BT → UI / external

Each `TagElement` contains:

- `id`: AprilTag ID (integer)
- `pose`: `PoseStamped` in a consistent frame (after post-processing and map transform)

Example usage in the UI:

```python
def _process_visible_tags(self, msg: TagElementArray):
    self.visible_tags = {tag.id: tag for tag in msg.elements}

def _process_reachable_tags(self, msg: TagElementArray):
    self.reachable_tags = {tag.id: tag for tag in msg.elements}
```

### 3.2 Command Buffer and Execution Status

- **Command buffer contents**
    - Topic: `fault_detector/command_buffer`
    - Type: `std_msgs/String`
    - Direction: BT → UI / external
    - Content: A textual representation of the queued commands.

- **Command execution status**
    - Topic: `fault_detector/command_tree_status`
    - Type: `std_msgs/String`
    - Direction: BT → UI / external
    - Content: High-level command-tree status (e.g. `IDLE`, `RUNNING MOVE_ARM_TO_TAG`, `FAILED START_SLAM`).

Example UI processing:

```python
def _process_buffer(self, msg: String):
    self.buffer_label.setText(f"Buffer: {msg.data}")

def _process_command_status(self, msg: String):
    self.command_status_label.setText(f"Command Status: {msg.data}")
```

---

## 4. Mapping and Navigation Interfaces

The mapping/navigation subsystem exposes simple topics used by the navigation controls UI.

### 4.1 Map, Waypoint, and Landmark State

- **Active map**
    - Topic: `/active_map`
    - Type: `std_msgs/String`
    - Direction: Mapping subsystem → UI

- **Map list**
    - Topic: `/map_list`
    - Type: `fault_detector_msgs/StringArray`
    - Direction: Mapping subsystem → UI
    - Content: `names[]` – list of map names.

- **Waypoint list**
    - Topic: `/waypoint_list`
    - Type: `fault_detector_msgs/StringArray`
    - Direction: Mapping subsystem → UI

- **Landmark list**
    - Topic: `/landmark_list`
    - Type: `fault_detector_msgs/StringArray`
    - Direction: Mapping subsystem → UI

The Qt UI uses these to populate dropdowns (`NavigationControls`).

### 4.2 Mapping and Navigation Commands

These are all sent via `fault_detector/commands/complex_command` with appropriate `command_id` and fields:

- `CREATE_MAP`
    - `map_name` – name for the new, empty map.

- `SWAP_MAP`
    - `map_name` – name of map to activate.

- `DELETE_MAP`
    - `map_name` – map to delete.

- `START_SLAM`, `START_LOCALIZATION`, `STOP_MAPPING`
    - No extra fields required beyond `command_id`.

- `ADD_CURRENT_POSITION_WAYPOINT`
    - `map_name` – active map.
    - `waypoint_name` – name for the waypoint.

- `DELETE_WAYPOINT`
    - `map_name`
    - `waypoint_name`

- `MOVE_TO_WAYPOINT`
    - `map_name`
    - `waypoint_name`

- `ADD_TAG_AS_LANDMARK`
    - `map_name`
    - `waypoint_name` – e.g. `"Tag_23"`.
    - `tag` – `TagElement` from the visible-tags output.

- `DELETE_LANDMARK`
    - `map_name`
    - `waypoint_name` – landmark name.

See `navigation_controls.py` for concrete examples of how the UI populates and sends these commands.

---

## 5. Recording and Playback Interfaces

The recording subsystem is controlled via a dedicated message type and exposes the list of available recordings.

### 5.1 Control Topic

- **Topic:** `fault_detector/record_control`
- **Type:** `fault_detector_msgs/CommandRecordControl`
- **Direction:** UI / external → recorder

`CommandRecordControl` fields:

- `name` (`string`) – recording name
- `mode` (`string`) – one of:
    - `"start"` – begin recording commands
    - `"stop"` – end recording
    - `"play"` – replay a recording by name
    - `"delete"` – delete an existing recording

Example from `recording_controls.py`:

```python
msg = CommandRecordControl()
msg.name = self.record_name_field.text().strip()
msg.mode = "start"  # or "stop", "play", "delete"
self.record_control_pub.publish(msg)
```

### 5.2 Recordings List Topic

- **Topic:** `fault_detector/recordings_list`
- **Type:** `fault_detector_msgs/StringArray`
- **Direction:** Recorder → UI / external
- **Content:** `names[]` – available recording names.

Example:

```python
def update_recordings_dropdown(self, msg: StringArray):
    self.recordings_dropdown.clear()
    self.recordings_dropdown.addItems(sorted(msg.names))
```

---

## 6. Typical Integration Recipe

To implement your own UI or agent:

1. **Connect to ROS 2** and create a node.
2. **Publish commands** on `fault_detector/commands/complex_command`:
    - Set `command.command_id` to the desired `CommandID`.
    - Add `tag`, `offset`, `map_name`, `waypoint_name`, `wait_time` as needed.
3. **Subscribe** to:
    - `fault_detector/state/visible_tags` and `fault_detector/state/reachable_tags` (for tag-aware behaviours).
    - `fault_detector/command_buffer` and `fault_detector/command_tree_status` (for execution state).
    - `/active_map`, `/map_list`, `/waypoint_list`, `/landmark_list` (for navigation GUIs).
    - `fault_detector/recordings_list` (if supporting recording playback).
4. (Optional) **Control recording** with `fault_detector/record_control`.

The existing Qt-based [`Fault_Detector_UI`](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fui_classes%2Ffault_detector_ui.py) is a reference client implementing all of the above interactions. Any alternative interface can be built by reusing the same topics and message types without changes to the underlying Behaviour Tree or robot-side logic.