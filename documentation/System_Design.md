# System Design Overview

GENERAL CONCEPT OF THIS DOCUMENT

INHALTSANGABE

## General Architecture

The developed system extends the Boston Dynamics Spot robot with a modular ROS 2–based framework for external fault detection and non-invasive condition
monitoring.  
Its architecture follows the principles of modularity, reusability, and transparency. Each subsystem, navigation, manipulation, data acquisition, and user
interaction, can operate independently while being orchestrated through a shared behavioral control layer.

At the center of the architecture is the Behavior Tree, which functions as the system’s control and coordination core.  
It continuously monitors incoming commands and system states, deciding which behavior to execute at any given time.  
The tree integrates sensing, decision-making, and action execution into a single hierarchical control structure.

Incoming user commands are received and buffered, after which the Behavior Tree interprets their intent and delegates them to the corresponding behavioral
branch.  
Each branch encapsulates a distinct capability of the robot, such as navigation, manipulation, or mapping, and defines the logical sequence of steps required to
complete that task.

This approach places all high-level decision logic inside the Behavior Tree itself, allowing it to directly coordinate perception, motion, and feedback
processes without external supervisory nodes.  
It also ensures that each command is executed deterministically, with built-in handling for cancellation, error conditions, and feedback publication.

The result is a modular yet unified control structure: new behaviors can be introduced by extending the tree with additional branches, while the communication
and data exchange between components remain standardized through ROS 2 topics, services, and the shared blackboard.

An architectural overview is provided as a diagram:
![general_architecture.png](images%2FSystem_Design%2Fgeneral_architecture.png)
*Figure: High-level software architecture of the ROS2 system developed.*

---

# Core Concept

The following Core Concept section provides a concise overview of the system’s main components as illustrated in the architectural diagram above.
It outlines the functional purpose of each major subsystem and explains how they interact within the overall control structure.
This section serves as an overview of the high-level architecture. The subsequent Detailed System Design elaborates on implementation details, data
handling, and internal logic.

## Behavior Tree Execution Layer

The Behavior Tree (BT) constitutes the logical center of the system.  
It defines the robot’s operational logic as a dynamic hierarchy of nodes, including conditions, actions, and control structures such as selectors and
sequences.  
This approach was chosen over traditional finite state machines to improve scalability, readability, and runtime flexibility.

Within the BT, sensor data and system states are continuously written to a shared blackboard, which acts as the central data exchange layer.  
All decision-making and command execution are based on this shared context.

The BT also incorporates a command-handling branch, which processes incoming commands in a first-in, first-out (FIFO) buffer to ensure sequential execution and
prevent conflicts.  
Each command is interpreted by the BT and mapped to its corresponding behavior subtree, which encapsulates the logical steps required to achieve the requested
action.

This modular and message-driven structure allows the system to:

- React to external inputs in real time.
- Integrate new actions without modifying core logic.
- Maintain synchronized state feedback across all subsystems

## System Components

### 1. User Interface (UI)

The User Interface provides an entry point for system control and monitoring.  
It allows the user to:

- Send commands (e.g., manipulator goals, navigation goals, scanning sequences).
- Manage recorded command sequences.
- View system state, queue contents, and behavior execution status in real time.

All commands issued via the UI are transmitted through the same communication pathway as recorded test commands, ensuring consistent handling between manual and
autonomous operation.

The UI is implemented as a loosely coupled subsystem with a focus on modularity, expandability, and testability.  
Its main purpose is to support experimental development and automated testing, where usability is secondary.  
It can be replaced by a simplified operator interface or integrated with an automated control agent without requiring structural changes to the rest of the
system.

### 2. Spot ROS 2 Driver Integration

The system interfaces with the Boston Dynamics Spot robot through the official **Spot ROS 2 driver**, which bridges the functionality of the Spot SDK to the ROS
2 ecosystem.  
This driver exposes all major robot capabilities as ROS 2 topics, services, and actions, allowing external control and monitoring without directly interacting
with the Spot-SDK.

The driver corresponds to **Spot SDK version 5.0.1** and provides access to both low-level sensor data and high-level control functions.  
Among the supported interfaces are:

- Base and arm motion control
- Gripper actuation (open/close, force control)
- Camera image and depth streams
- Localization and state feedback
- Power and lease management
- Autonomous mobility and obstacle avoidance

All commands issued by the system, whether navigation goals, arm trajectories, or gripper actions are ultimately executed through this driver.  
It ensures safety and compliance with Spot’s onboard controllers, which continue to handle balance, terrain adaptation, and obstacle avoidance internally.  
This design allows external behavior coordination and automation while maintaining the robot’s built-in safety features.

A detailed reference of the Spot ROS 2 driver and available interfaces can be found here:  
[https://github.com/boston-dynamics/spot_ros2](https://github.com/boston-dynamics/spot_ros2)


---

### 3. Command Subtree

The command subtree handles the mapping of incoming commands to the appropriate execution sequences.  
Within the behavior tree, it serves as the decision layer that activates navigation, manipulation, or system management behaviors.  
Each command type is implemented as an independent subtree to maintain separation of concerns and allow isolated testing.

#### 3.1 Navigation Control

The Navigation Commands are responsible for controlling the Spot base, handling maps, and maintaining localization.  
This subsystem builds upon the ROS 2 navigation stack (`nav2`) and the official Spot ROS 2 driver, integrating autonomous motion planning with the robot’s
internal control and safety systems.

Navigation functionalities can be divided into two main categories: **Mapping and Map Navigation** and **Base Control**.

##### Mapping and Map Navigation

The system supports both the creation of new maps and the use of previously stored ones.  
Key functions include:

- **Map creation and SLAM:** Generating new maps using `rtabmap_ros` and Spot’s onboard odometry and vision data.
- **Map saving and loading:** Storing generated maps for later use, allowing the robot to re-localize within previously explored environments.
- **Waypoint management:** Users can define, name, and store specific locations in a map. These waypoints can later be selected as goal positions for autonomous
  navigation.
- **Path planning:** High-level path planning is executed through the ROS 2 navigation framework, ensuring collision-free paths between waypoints or
  user-defined targets.

Mapping and navigation are coordinated through the Behavior Tree’s command handling subsystem.  
The robot can autonomously transition between mapping, navigating, and inspection modes without manual reconfiguration.

##### Base Control

General base control handles direct motion commands such as standing, sitting, walking, or rotating.  
These commands are executed through the Spot ROS 2 driver’s base control interfaces, which internally ensure terrain adaptation, stability, and obstacle
avoidance.

The external system therefore does **not** override Spot’s native locomotion safety or balance algorithms.  
Instead, it issues high-level target poses while Spot’s onboard systems handle:

- Real-time obstacle avoidance,
- Leg coordination and gait stability,
- Dynamic balance and posture control.

Base motion commands can be specified relative to the robot’s current position or relative to a detected marker (e.g., an AprilTag).  
This allows the system to move precisely in relation to the environment or objects of interest, enabling tasks such as approaching a tag or aligning for
manipulation.

This layered control approach enables robust, high-level navigation planning within the ROS 2 ecosystem while preserving Spot’s internal safety and mobility
features.

#### 3.2 Manipulation Control

The Manipulation Commands handle the Spot Arm and the attached prototype sensor head.  
Using ROS 2 motion planning tools, the system enables Cartesian and joint-space control for surface scanning and interaction tasks.  
Commanded poses similar to base control can be defined through:

- Relative positional input, or
- Marker-based detection (AprilTags).

A software-level emergency stop mechanism is integrated at this layer.  
The behavior tree monitors a shared blackboard flag and, if triggered, immediately cancels all ongoing actions, stops base motion, halts mapping activities, and
moves the manipulator into a stowed position.  
This ensures safety overrides take effect at any point in the command sequence.

---

### 4. Sensing Subtree

The sensing subsystem forms one of the main branches of the behavior tree alongside command execution and feedback publishing.  
It continuously gathers environmental and system state information, processes it, and writes results to the shared blackboard.

The sensing branch runs multiple non-blocking tasks in parallel, including:

- AprilTag detection using both hand-mounted and body-mounted cameras.
- Transformation of detected tag poses into the world frame.
- Continuous subscription to localization and state updates from the Spot driver.
- Monitoring of new incoming commands from the command subscriber.

All sensing data are published to the blackboard for access by other components.  
Feedback from ongoing actions, such as navigation progress, manipulation status, and error reports, is continuously published to the UI and recording systems,
ensuring synchronized operation and traceable execution across all active subsystems.

#### AprilTag detection and arm-camera integration

Spot’s driver already contributes fiducial detections from the body cameras into the system transform tree. However, the arm-mounted camera is not used. To
support precise manipulator-centric workflows and to enable the robot to use reference points outside the field of view of the body cameras, the system
integrates an additional AprilTag detection pipeline based on the [apriltag_ros](https://github.com/christianrauch/apriltag_ros)
package.

Integration summary:

- body-camera fiducials (provided by the Spot driver) and arm-camera detections (provided by apriltag_ros) are treated as complementary inputs to the perception
  subsystem;
- apriltag_ros performs tag detection on the arm camera stream and publishes tag poses so they appear in the same spatial frame conventions used elsewhere in
  the system.

The "AprilTag Detector" element in the architecture diagram should therefore be understood as a combined perception subsystem: the Spot driver’s fiducial
outputs plus the apriltag_ros node operating on the arm camera. This combined setup enables robust, manipulator-aware localisation and alignment for scanning
tasks. See apriltag_ros for implementation details: https://github.com/christianrauch/apriltag_ros

### 5. Feedback Subtree

The Feedback Subtree provides real-time status information to the user interface and any other monitoring nodes.  
It subscribes to key topics in the system and updates of the blackboard and publishes that information for external monitoring.

**Main Functions:**

- **Visible Tags:** subscribes to `fault_detector/state/visible_tags` to track tags currently detected by the robot.
- **Reachable Tags:** subscribes to `fault_detector/state/reachable_tags` to determine which tags can be interacted with.
- **Command Buffer:** subscribes to `fault_detector/command_buffer` to reflect queued commands awaiting execution.
- **Command Tree Status:** subscribes to `fault_detector/command_tree_status` to monitor current action execution state.

The feedback branch operates in parallel to command execution and sensing, ensuring the UI always reflects the latest system state without blocking ongoing
operations.  
This allows the operator to make informed decisions, verify system behavior, and respond quickly to errors or changes in the environment.

### 6. Command Recording and Playback

The Recording and Playback Node provides functionality to capture and reproduce command sequences.  
During operation, all commands published to the execution pipeline are timestamped and logged.  
These sequences can later be replayed for regression testing, demonstrations, or reproducible condition monitoring, using the same command-handling structure as
live operation.

This mechanism allows complex inspection routines or test scenarios to be executed repeatedly without additional programming, turning the system into a
scriptable robotic test platform.

---

## Data Flow Summary

1. **External Input (UI / API)** → Publishes a command to the system topic.
2. **Command Subscriber** → Queues and validates incoming commands.
3. **Behavior Tree Execution** → Maps command ID to subtree and executes associated behavior.
4. **Subsystem Nodes** (Navigation, Manipulation) → Carry out low-level control.
5. **Blackboard** → Stores sensor data, intermediate results, and execution state.
6. **Feedback Publishing** → UI continuously receives updates for visualization.

This data-driven, modular organization ensures a clear separation of responsibilities while maintaining robust synchronization across components.

The UI is intentionally loosely coupled to the rest of the system, prioritizing expandability and testability over end-user usability.  
It serves primarily as a research and development interface, designed for scientific testing and system validation rather than field deployment.  
However, this design allows it to be easily replaced by more user-friendly interfaces or even autonomous control agents (e.g., AI-based assistants or
voice-driven operators) without altering the underlying execution framework.

---

## Design Rationale

- **ROS 2 Integration:**  
  ROS 2 provides distributed communication via topics, services, and actions, enabling flexible orchestration between Spot’s hardware interfaces and custom
  behavior logic.

- **Behavior Trees over Finite State Machine:**  
  Behavior Trees offer a hierarchical and reactive control framework, simplifying parallel behaviors, condition checking, and command prioritization.

- **Centralized Blackboard:**  
  The blackboard serves as a single source of truth for shared data, reducing inter-node dependencies and easing debugging.

- **Record & Playback:**  
  Enables repeatable testing, benchmarking, and demonstration of inspection routines, which is critical in research contexts and aligns with project
  deliverables defined in the Requirements Specification.

---
Overall the system’s modular design enables seamless interaction between autonomous decision-making and manual control while remaining flexible for future
sensor integration and algorithmic extensions.  
Each subsystem can operate independently or as part of the full execution pipeline, ensuring both scalability and maintainability core design goals stated in
the Requirements Specification.

# Detailed System Design

This section explains how each part of the system is implemented and interacts internally, bridging the gap between high-level architecture and operational
behavior. It focuses on subsystems, internal data handling, software dependencies, and execution logic.

> **Note:** In this section, "nodes" refer to behavior tree nodes, which are logical elements of the Behavior Tree (actions, conditions, and control-flow
> structures). They are distinct from ROS 2 nodes, which are the software processes that implement the system’s functionality and communicate via topics,
> services, or actions.

In a typical ROS 2 + py_trees setup:

- The Behavior Tree is hosted within a single ROS 2 node.
- Each tree-node is a Python object and may:
    - Publish or subscribe to ROS 2 topics
    - Call ROS 2 services
    - Use ROS 2 timers
- All tree-nodes execute within the context of the host ROS 2 node, meaning they share the same process and runtime environment.

The following subsections describe the custom components and their interactions that form the developed control system.  
However, the design and internal functionality of major third-party components such as **spot_ros2**, **apriltag_ros**, **nav2**, and **rtabmap_ros** are not
explained in detail here, as each of these frameworks provides its own comprehensive documentation authored and maintained by their respective creators.  
References to these original sources are provided in the **References** (ADD REFERENCE HEREEEEEEEE) chapter at the end of this document.

This section therefore focuses on the integration logic, the data flow between components, and how the system builds upon these established frameworks to
achieve coordinated control, perception, and feedback.

## 1. Behavior Tree Implementation

The behavior of the system is organized and executed through a hierarchical Behavior Tree (BT) implemented with the `py_trees` and `py_trees_ros` frameworks.
This approach provides modularity, scalability, and clarity in defining complex robot behaviors by composing them from smaller, reusable building blocks.

The main entry point of the Behavior Tree is defined in the `bt_runner.py` file. The system runs as a single ROS 2 node (`bt_driver`), hosting the entire tree.
Within this node, individual tree-nodes (actions, checks, and control structures) are defined as Python objects that interact with the rest of the ROS 2 system
through topics, services, and actions.

The root node of the tree is a **Parallel** composite titled `"FaultDetectorSpot"`, which contains three major subtrees:

- **Sensing Tree**
- **Buffered Command Tree**
- **Publisher Tree**

Each of these serves a distinct purpose in the overall data and control flow.
The separation into three subtrees is used for clarity and modularity:

- The **Sensing Tree** handles all inputs into the system whether sensor readings, localization data, or user commands.
- The **Buffered Command Tree** manages decision-making and action execution based on these inputs.
- The **Publisher Tree** ensures consistent outward communication of system states and updates.

This division mirrors the natural flow of information through the system, from perception to decision to communication, allowing each subsystem to operate
independently while remaining synchronized through shared blackboard variables and ROS 2 communication channels.

The Subtree structure can be seen in the diagram built using [draw.io](https://www.drawio.com/) below, with notation adapted
from [Behavior Trees in Robotics and AI](https://arxiv.org/abs/1709.00084): An
Introduction by Michele Colledanchise &
Petter Ögren. This notation will be used consistently in the following chapters to illustrate each subtree in more detail.
> **Note:** The `py_trees` `Selector` node used in the implementation is slightly different from the classical Fallback node in the source. In `py_trees`,
> a `Selector` can optionally maintain memory of the last running child (with `memory=True`) and will resume from that child on subsequent ticks, whereas the
> standard Fallback always evaluates children from left to right on every tick. In the current system, this memory feature is **not utilized**, so
> the `Selector`
> behaves effectively like a classical Fallback node.

![bt_general_diagram.png](images%2FSystem_Design%2Fbt_general_diagram.png)
*Figure: The general behaviour tree structure and its division into three subtrees.*

## 2. Command Handling

The command-handling system defines how external input, such as user interface actions or automated test sequences, are received, interpreted, and transformed
into behavior tree-compatible actions.

A *command* represents an instruction for the system to perform a specific operation, such as moving the robot base, manipulating the arm, or executing a
scanning procedure. These commands can be broadly divided into two categories: **Basic Commands** and **Complex Commands**.

Basic Commands are minimal instructions that only contain a command identifier. They represent discrete, self-contained operations like starting or stopping a
behavior, toggling a component state, or triggering an emergency stop. Complex Commands, in contrast, can carry additional contextual information. This may
include spatial offsets, tag references, orientation modes, timing parameters, or mapping context. Complex commands enable parameterized control and are
represented by separate ROS 2 message types that expand on the BasicCommand structure.

Although multiple specialized message types could have been defined for each command variation, this approach was intentionally avoided. While having distinct
message definitions would provide stronger type safety, it would also increase the development effort required when introducing new command types. The chosen
design enables flexibility and rapid extension. Once the final command set is known, this trade-off
may be reconsidered as high modularity of the system enables swapping out the command handler without altering each command.

Incoming commands are handled by the **CommandSubscriber** behavior tree node. This node subscribes to two custom ROS 2 topics found in
the [fault_detector_msgs package](https://github.com/heini208/fault_detector_msgs): one for basic commands and one
for complex commands. Both types are queued internally upon reception, time-sorted, and then converted into internal command objects. These are stored appended
onto a command buffer on the
system’s blackboard, where they can be accessed by other tree components responsible for execution.

#### The Command Subscriber

The Command subscriber is a node from the sensing subtree it distinguishes between commands by inspecting their message type and the `commandID` parameter.  
The mapping from command identifiers to executable command objects is handled through builder functions. Each recognized command ID has a corresponding handler
that defines how it should be instantiated. This modular structure ensures that new commands can be integrated with minimal modification to the core system.

This version of the system also includes generic command handlers that map commands without their own class to a composite `generic_complex_command` type or, in
the case of basic commands, to simple command objects.  
The existence of a generic command class is primarily intended to enable quick prototyping of new command ideas. Ideally, every command would eventually have
its own command class and a dedicated subscriber handler to avoid fetching unnecessary parameters, increase type safety, and reduce the risk of errors from
handling unknown or partially defined commands.

The command subscriber also demonstrates how certain commands can be combinations of other commands. Combination commands are `commandID`s that represent
sequences of multiple low-level commands. The subscriber parses these messages and appends ordered sets of executable sub-commands to the buffer. For example, a
combination command might instruct the arm to move to a tag position, wait for a specific duration, and then return to a stowed pose. This sequence is
automatically generated inside the subscriber based on the contextual information contained in the incoming message.

Commands may include references to specific AprilTags, positional offsets, or predefined waypoints. This enables context-aware motion such as moving relative to
the robot’s current position, aligning to a marker, or executing actions in relation to known spatial features.

Finally, the command-handling layer integrates safety logic. Emergency-stop commands are recognized at any time and immediately clear the command buffer, halt
ongoing activity, and trigger the system’s stop routines. This ensures that user- or system-level interruptions are reliably enforced regardless of the current
execution state.

Through this structure, the command-handling subsystem forms the interface between high-level user or test input and the low-level behavior execution tree. It
abstracts the input modality, manages command sequencing, and maintains extensibility while preserving consistent execution semantics.

A detailed command reference table is provided later in this document.  
For extended per-command explanations, refer to the dedicated **Command Reference Document**, which contains argument structures, parameter options, and example
use cases.

> **Note on Current Limitation**:
> The system presently uses separate ROS 2 subscribers for Basic Commands and Complex Commands, even though both forward incoming messages into the same
> callback
> and unified processing pipeline. When commands of both types are published in quick succession, for example in recorded playback scenarios, their relative
> order
> may occasionally become inconsistent. This has so far only been observed during recorded message replay but not during live operation.

> As a temporary workaround, all Basic Commands can be encapsulated within Complex Command messages. This ensures that the full sequence of operations is
> received
> through a single subscriber path and therefore preserves ordering.  
> This limitation will be resolved once the Basic Command type is either removed or merged into a unified command structure.

### Buffered Command Subtree

#### Command Execution Flow

Once commands have been received, validated, and converted into their internal representations by the CommandSubscriber node, they are placed into a shared
command buffer on the system’s blackboard. At this point the command-handling layer has completed its responsibility: commands are normalized, ordered, and
ready for execution, but not yet acted upon.

The actual execution and life-cycle management of these command objects is handled by a dedicated subtree inside the behavior tree: the **Command Buffer Handler
**.  
This subsystem continuously monitors the command buffer, selects the next executable command, and invokes the appropriate action nodes responsible for base
movement, manipulation, mapping, recording, or other high-level functions.

The next section describes how the Command Buffer Handler operates, how command classes transition from buffered objects to active behaviors, and how execution
flow, cancellation, and completion states are coordinated. It also includes a summary of all currently available command types integrated into the system.

The Buffered Command Tree manages and executes user commands in a controlled, buffered manner. Commands received from the UI are stored, queued, and processed
one at a time. This design ensures that actions do not overlap and that the robot maintains a predictable execution flow.

The main structure includes:

- **CommandManager:** Acts as a command buffer, storing and managing incoming commands.
- **NewCommandGuard:** Ensures that only new commands trigger execution and that they can be safely interrupted.
- **Emergency Sequence:** Cancel Sequence Command that stops all robot movements.
- **Command Selector:** Selects the corresponding behaviour for each command ID popped from the buffer.
- **EmergencyGuard:** Enables cancellation of running commands through the estop_flag blackboard variable.
- **Command Execution Nodes:** Behaviours responsible for handling the execution of commands

Each supported command, such as `MOVE_ARM_TO_TAG`, `START_SLAM`, or `DELETE_MAP` is represented by a corresponding behavior or subtree that defines its logic
and
interactions with the rest of the system.

The buffered command subtree is visualized in the following behaviour tree diagram:

![command_subtree.diagram.png](images%2FSystem_Design%2Fcommand_subtree.diagram.png)
*Figure: The Buffered Command Subtree. Shows the command buffer, guards, emergency paths, and selector logic.*

#### CommandManager

The [CommandManager](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Futility%2Fcommand_manager.py) is responsible for buffering incoming commands and
dispatching them to the command execution tree in a controlled manner. Its main functions are:

- **FIFO Dispatch**: When the command tree is idle (i.e., not `RUNNING`), the oldest command is popped from the buffer and written to `blackboard.last_command`.
- **Emergency Handling**: If an `EMERGENCY_CANCEL` command is found anywhere in the buffer, it is immediately promoted to `last_command` and the buffer is
  cleared. This ensures that emergency stops are enforced reliably.
- **Failure Handling**: Optionally, commands that fail (`FAILED` status) can trigger buffer clearing to prevent the execution of outdated or conflicting
  commands.

#### NewCommandGuard

The [NewCommandGuard](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Futility%2Fnew_command_guard.py) ensures that each command is processed exactly once,
avoiding duplicate execution:

- **Timestamp Check:** Compares the timestamp of `last_command` with `last_processed_command`.
- **Activation Control:** Only allows the guarded command sequence to execute if a new command is detected.
- **Duplicate Prevention:** Prevents repeated executions of the same command.

#### EmergencyGuard and Emergency Sequence

Because the CommandManager cannot terminate nodes directly, the EmergencyGuard decorator wraps a command sequence and immediately cancels it if an
emergency flag is set:

- **Immediate Emergency Handling:** Enables immediate execution of `EMERGENCY_CANCEL` sequences by canceling currently running commands.
- **Safe Stop:** Ensures the robot can safely stop any ongoing action regardless of current command execution.

#### Command Selector

The **Command Selector** is the core node in the behaviour tree responsible for choosing and executing the correct command based on the current `last_command`
from the blackboard.

Key points:

- Implemented as a `py_trees.composites.Selector` that **checks each command ID** and runs the corresponding behaviour sequence.
- Wraps each command in a **guarded sequence** that first verifies the blackboard variable `last_command` matches the command ID.
- Handles both **Spot actions** (via `ActionClientBehaviour` / `SimpleSpotAction`) and **non-robot commands** (navigation, mapping, or state updates).
- Integrated with **emergency cancellation** via a higher-level `CancelableCommandSelector` that prioritizes emergency stop sequences over normal commands.
- Supports **preprocessing and helper logic**, e.g., fetching target poses, checking tag visibility, or computing SLAM-related goals before executing the
  command.
- Modular: Adding a new command only requires defining its behaviour sequence and registering it in the selector.

### Command Behaviour Execution Nodes

When adding a new command to the tree, each command is automatically wrapped with a Command ID checker by a helper class. Each command needs this checker for
the
Command Selector to ensure that the correct subtree or action sequence is executed for the currently active command stored on the blackboard.

**Executors of commands** can be broadly divided into two categories based on how they interact with the robot:

#### 1. Action Client Behaviours [(`ActionClientBehaviour` / `SimpleSpotAction`)](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Futility%2Fspot_action.py)

These executors communicate **directly with the robot via the Spots ROS2 drivers `RobotCommand` action**. They handle asynchronous action sending, monitoring,
and cancellation.

###### Characteristics:

- Inherit from **`ActionClientBehaviour`** (custom Spot action lifecycle) or **`SimpleSpotAction`** (Spot-specific `RobotCommand` helper).
- **Lifecycle Management:** Handles initialization, goal sending, result polling, and cleanup automatically.
- **Emergency Handling:** Can cancel ongoing goals immediately if an emergency is triggered.
- **Blackboard Integration:** Reads the current command from the py_trees blackboard set by the command manager (`last_command`) to set necessary parameters.
- **Extensible:** Subclasses must implement:
    - `_build_goal() → Goal`: Construct the goal message.
- **Optional Overrides:** The following methods have default implementations in the base class and **do not need to be implemented by most subclasses**. Only
  override if special logic is required:
    - `_init_client() → bool`: Initialize the specific action client.
    - `_send_goal(goal) → Future`: Send the goal via the action client.
- **Spot-Specific Variant:** `SimpleSpotAction` wraps Spot's `RobotCommand` action, providing helpers to simplify sending robot-specific commands.
- **Examples of Subclasses:**
    - `StowArmActionSimple`
    - `ReadyArmActionSimple`
    - `CloseGripperAction`
    - `ManipulatorMoveArmAction`
    - `ManipulatorMoveRelativeAction`
    - `BaseMoveToTagAction`

#### 2. Other Commands (`py_trees.behaviour.Behaviour`)

Some commands do **not** use the ROS2 `RobotCommand` action driver. Instead, they interact with:

- **Mapping or SLAM** (e.g., RTAB-Map)
- **Navigation** (e.g., Nav2)
- **Information flow** or **blackboard state**

###### Key Points

- Inherit directly from `py_trees.behaviour.Behaviour`.
- Implement `update()` to define their specific logic.
- May read from and/or write to blackboard variables.
- Communicate via **topics**, **services**, or **helper classes**, rather than `RobotCommand` actions.

###### Examples

- `DeleteWaypoint` — removes a waypoint from the map.
- `NavigateToGoalPose` — sends a goal to Nav2.
- `ManipulatorGetGoalTag` — fetches a visible goal tag.

### Available Commands

Below is a list of all currently available commands and their respective behaviours. Each command is referenced via its `CommandID` from
the [command_ids.py](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fcommands%2Fcommand_ids.py) file. The table shows the mapping from command IDs to the
corresponding behavior sequences.

The exact functionality of each command will become clear either:

- In the dedicated section describing the corresponding subsystem, or
- By reading the detailed explanation in the [Command Reference Document](detailed_command_descriptions.md) that provides argument structures, parameter options, and example use cases for each
  command.

| CommandID                     | Behavior Sequence                                                                                                                                                                                                                                       |
|-------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| STOW_ARM                      | [StowArmActionSimple](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fmanipulation%2Fstow_arm_action.py)                                                                                                                                            |
| READY_ARM                     | [ReadyArmActionSimple](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fmanipulation%2Fready_arm_action.py)                                                                                                                                          |
| TOGGLE_GRIPPER                | [ToggleGripperAction](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fmanipulation%2Ftoggle_gripper_action.py)                                                                                                                                      |
| CLOSE_GRIPPER                 | [CloseGripperAction](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fmanipulation%2Fclose_gripper_action.py)                                                                                                                                        |
| MOVE_ARM_TO_TAG               | [ManipulatorGetGoalTag](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fmanipulation%2Fmanipulator_get_goal_tag.py) → [ManipulatorMoveArmAction](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fmanipulation%2Fmanipulator_move_arm_action.py) |
| MOVE_BASE_TO_TAG              | [BaseGetGoalTag](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fnavigation%2Fmove_base%2Fbase_get_goal_tag.py) → [BaseMoveToTagAction](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fnavigation%2Fmove_base%2Fbase_move_to_tag_action.py)    |
| MOVE_ARM_RELATIVE             | [ManipulatorMoveRelativeAction](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fmanipulation%2Fmanipulator_move_relative_action.py)                                                                                                                 |
| MOVE_BASE_RELATIVE            | [BaseMoveRelativeAction](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fnavigation%2Fmove_base%2Fbase_move_relative_action.py)                                                                                                                     |
| STAND_UP                      | [StandUpActionSimple](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fnavigation%2Fstand_up_action.py)                                                                                                                                              |
| WAIT_TIME                     | [WaitForDuration](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Futility%2Fwait_for_duration.py)                                                                                                                                                   |
| STOP_BASE                     | [PublishZeroVel](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fnavigation%2Fcancel_movement.py)                                                                                                                                                   |
| START_SLAM                    | [EnableSLAM](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fmapping%2Fenable_slam.py)                                                                                                                                                              |
| START_LOCALIZATION            | [EnableLocalization](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fmapping%2Fenable_localization.py)                                                                                                                                              |
| CREATE_MAP                    | [InitializeEmptyMap](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fmapping%2Finitialize_empty_map.py)                                                                                                                                             |
| DELETE_MAP                    | [DeleteMap](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fmapping%2Fdelete_map.py)                                                                                                                                                                |
| SWAP_MAP                      | [SwapMap](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fmapping%2Fswap_map.py)                                                                                                                                                                    |
| STOP_MAPPING                  | [StopMapping](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fmapping%2Fstop_mapping.py)                                                                                                                                                            |
| ADD_CURRENT_POSITION_WAYPOINT | [SaveCurrentPoseAsGoal](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fnavigation%2Fsave_current_pose_as_goal.py) → [AddGoalPoseAsWaypoint](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fnavigation%2Fadd_goal_pose_as_waypoint.py)         |
| ADD_TAG_AS_LANDMARK           | [SetTagAsGoal](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fnavigation%2Fset_tag_as_goal.py) → [AddGoalPoseAsLandmark](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fnavigation%2Fadd_goal_pose_as_landmark.py)                            |
| DELETE_WAYPOINT               | [DeleteWaypoint](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fmapping%2Fdelete_waypoint.py)                                                                                                                                                      |
| MOVE_TO_WAYPOINT              | [SetWaypointAsGoal](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fnavigation%2Fset_waypoint_as_goal.py) → [NavigateToGoalPose](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fnavigation%2Fnavigate_to_goal_pose.py)]                        |
| DELETE_LANDMARK               | [DeleteLandmark](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fmapping%2Fdelete_landmark.py)                                                                                                                                                      |

---

## 3. Sensing Subtree

The Sensing Tree handles all incoming data and system inputs. This includes sensor data (e.g., detected AprilTags, robot poses) and, as already mentioned, user
commands from the UI. Its main goal is to keep the system aware of its surroundings and incoming requests in real time.

As shown in the **Sensing Tree overview diagram** below (see Figure),  
the Sensing Subtree runs several behaviors in parallel, including:

- [**CommandSubscriber:**](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fsensing%2Fcommand_subscriber.py) Receives new commands from the user interface.  
  For details on how command messages are parsed and validated before entering the selector,
  see the dedicated section: [The Command Subscriber](#the-command-subscriber).
- [**Localization Pose Subscriber:**](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fsensing%2Flast_localization_pose.py) Subscribes to and stores the robot’s most recent localization pose.
- **ScanForTags Sequence:** A nested sequence that manages tag detection through both the Spot’s built-in cameras
  and the arm-mounted hand camera.

The tag detection process combines multiple components:

- [**DetectVisibleTags**](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fsensing%2Fdetect_visible_tags.py) and [**HandCameraTagDetection**](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fsensing%2Fhand_camera_tag_detection.py) are responsible for scanning for AprilTags.
- [**CheckTagReachability**](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fsensing%2Fcheck_tag_reachability.py) determines whether a detected tag is reachable.
- [**VisibleTagToMap**](..%2Ffault_detector_spot%2Fbehaviour_tree%2Fnodes%2Fsensing%2Fvisible_tag_to_map.py) transforms the detected tag poses into the global SLAM map frame if mapping is used.

The AprilTag detection is based on both the Spot driver’s integrated fiducial detection (which publishes AprilTags
directly into the TF tree) and the external [`apriltag_ros`](https://github.com/AprilRobotics/apriltag_ros) package.
The additional use of [`apriltag_ros`](https://github.com/AprilRobotics/apriltag_ros) is necessary since the Spot driver does not include the arm-mounted camera
in
its fiducials detection. This allows the arm to move independently, scan for reference tags beyond the main camera’s field
of view, and then navigate to precise scanning points.
<a id="bt-overview"></a>
![sensing_subtree_diagram.png](images/System_Design/sensing_subtree_diagram.png)

*Figure: The Sensing Subtree, running tag-sensing, localization, and command reception behaviors in parallel.*

### Tag Handling

While most components of the Sensing Subtree are already explained or relatively self-explanatory, the **tag-scanning pipeline** requires additional detail.  
This is because it integrates two independent detection sources, multiple coordinate frames, and a fusion of visual and depth data.  
The following section explains how fiducials are detected, transformed, and enriched before being used by the Behavior Tree.

#### AprilTag Type and Generation

The system uses **AprilTags of the 36h11 family**, a robust and widely used encoding suitable for localization, mapping, and manipulation.

An example configuration and usage snippet is provided in the appendix (see [Appendix 1: Example AprilTag (ID: 0)](#appendix-1-example-apriltag-id-0))

All tags used in this project were generated using the online [AprilTag generator](https://chaitanyantr.github.io/apriltag.html)

AprilTags are used because they provide reliable, uniquely identifiable visual fiducials that remain detectable even under challenging lighting,
motion, or viewing-angle conditions.
The 36h11 encoding is widely supported by both Boston Dynamics’ built-in fiducial detector and by `apriltag_ros`, ensuring compatibility across all parts of the
system.

#### Spot’s Integrated Fiducial Detector

Spot’s built-in detector identifies AprilTags using the robot’s body cameras.  
For each detected tag, Spot automatically publishes:

- a TF frame for the tag (e.g., `fiducial_23`),
- a fully estimated 6-DoF pose (position + orientation),
- transforms linking it to the robot’s internal frames.

Because these frames are inserted directly into Spot’s TF tree, they can be transformed into any connected frame, such as`body`, `odom` or `world`

A visualization of the TF structure is included in the appendix  
(see [Appendix 2: TF Tree for Spot AprilTag Detections](#appendix-2-tf-tree-for-spot-apriltag-detections)).

However, Spot’s detector **does not include the manipulator’s hand camera** in its TF hierarchy.  
Tags outside the body-camera field of view therefore require an external detector.

#### Hand-Camera Tag Detection (apriltag_ros + Depth Fusion)

To enable tag scanning using the manipulator’s wrist-mounted camera, the Sensing Subtree implements a second detection method based on the `apriltag_ros` package.

The processing pipeline works as follows:

1. **apriltag_ros** detects AprilTags in the hand-camera image stream.
2. It computes an initial tag pose, but this estimate **lacks correct depth (Z) information** because:

- the depth stream of the arm camera is not integrated into `apriltag_ros`, and
- the AprilTag pose estimator cannot recover real-world scale from a monocular image alone.

3. To recover accurate 3D information, the system:

- determines the pixel location of the detected tag,
- retrieves the depth value at that pixel from the manipulator’s depth image, and
- **fuses** the depth measurement with the AprilTag’s 2D pose estimate.

4. The corrected 3D tag pose is then published into the system and added to the TF tree, enabling transformation into map frames such as `odom` and `world`.

This fusion step ensures that hand-camera detections provide **complete, metric 3D tag poses** equivalent in usability to Spot’s built-in detector.  
It enables precise alignment, manipulation, and navigation even when tags are only visible to the arm camera.

#### Tag Post Processing

After AprilTags are detected by the Sensing Subtree, a post-processing step evaluates each tag for its usability in downstream tasks. This processing serves two main purposes:

1. **Reachability Check:**  
   Each detected tag is checked to determine whether it falls within the circular reach of the robot’s manipulator. This ensures that:
  - Manipulation behaviours do not attempt to reach tags that are physically out of range.
  - The user interface can provide feedback on which tags are currently reachable versus just visible.

   Tags that pass this check are stored in a **`reachable_tags`** list on the blackboard, in addition to the **`visible_tags`** list maintained by the Sensing Subtree.

2. **Map Frame Transformation:**  
   To simplify further processing, a transformer node converts detected tag poses into the global map frame. This allows any behaviour or node requiring map coordinates of tags to access them directly, without needing to perform repeated transformations.

By separating these steps, the system ensures efficiency, safety, and clear feedback to the user while maintaining modularity in the behaviour tree.

---

## 4. Feedback Subtree

### Publisher Tree

The Publisher Tree is responsible for continuously publishing status information to external components, particularly the UI and SLAM system. It runs parallel
to all other subsystems and ensures that important state data is always available.

Its main elements include:

- **PublishInitialUIInfoOnce:** Sends initial UI information when the system starts.
- **CommandStatusPublisher:** Reports command execution states and system status updates.
- **LandmarkRelocalizer:** Publishes localization updates to SLAM, assisting in map alignment and relocalization.

## 4. Recording and Playback

## 5. Subsystem Interactions

## 6. User Interface

- Loosely-coupled architecture and communication with the behavior tree
- Command input and visualization of system state
- Integration with recording/playback
- Optional replacement with automated control agents
- devided into manip,navigation,mapping

## 7. Technology Stack

- ROS 2 version and packages
- py_trees for behavior tree management
- Spot SDK and drivers
- Other relevant libraries and tools

# Appendix 

### Appendix 1: Example AprilTag (ID: 0)
![tag36h11-0.svg](images/System_Design/tag36h11-0.svg)  
*Figure: AprilTag with 36h11 encoding, ID 0.*

### Appendix 2: TF Tree for Spot AprilTag Detections
[tf_tree_spot_tag.pdf](images/System_Design/tf_tree_spot_tag.pdf)  
*PDF: TF tree showing how Spot publishes AprilTag detections relative to body, odom, and world frames.*


