# Fault Detector Spot – ROS 2 Behaviour-Tree Control for Boston Dynamics Spot

This repository contains the ROS 2 implementation of the **Fault Detector Spot** system:  
a behaviour‑tree–based control stack for the Boston Dynamics Spot robot, combining:

- High‑level base and manipulator control
- AprilTag‑based perception
- RGB‑D SLAM (RTAB‑Map) and Nav2 navigation
- Command recording & playback
- A PyQt5 GUI for development and experimentation

The system was developed as part of a **Master’s project at Hochschule Bonn‑Rhein‑Sieg**,  
in cooperation with **Fraunhofer IAO**, by **Marcel Stemmeler**.

For the full technical description, refer to the accompanying system design document:  
[`docs/System_Design.md`](System_Design.md).

---

## 1. Repository Structure (high level)

- `fault_detector_spot/`
  - `behaviour_tree/`
    - `bt_runner.py` – main behaviour tree node
    - `nodes/…` – custom BT behaviours for sensing, mapping, navigation, manipulation, utility
    - `commands/…` – internal command classes and `CommandID` definitions
    - `ui_classes/…` – UI and recording control nodes
- `launch/`
  - `fault_detector_launch.py` – main launch file for real robot
  - `sim_fault_detector_launch.py` – simplified launch for simulation
  - `nav2_spot_launch.py` – Nav2 bringup tuned for Spot
  - `rtab_mapping_launch.py` – RTAB‑Map launch (mapping & localization)
- `config/`
  - `nav2_spot_params.yaml` / `nav2_sim_params.yaml` – Nav2 configuration
  - `mapping.rviz` – RViz config for mapping/navigation
  - `my_tags.yaml`, `my_tags_sim.yaml` – AprilTag configuration for `apriltag_ros`
- `docs/`
  - `System_Design.md` – detailed system design & architecture (you pasted the latest version)
  - Additional docs, figures and example recordings under `images/System_Design/…`

---

## 2. Core Dependencies

### 2.1 ROS 2 & Robot

- **ROS 2 Humble Hawksbill** (recommended)
- **Boston Dynamics Spot** with:
  - Spot SDK 5.0.1 (via `spot_ros2`)
  - Optional manipulator arm (required for manipulation functions)
  - Body cameras (required), hand camera recommended

### 2.2 Packages from this ecosystem

You must have the following ROS 2 packages installed and sourced:

- **Robot & Messages**
  - [`spot_ros2`](https://github.com/bdaiinstitute/spot_ros2) – official Spot ROS 2 driver
  - `spot_msgs`, `bosdyn_msgs`, `spot_wrapper`, `spot_common`, `synchros2` (as required by `spot_ros2`)
  - [`fault_detector_msgs`](https://github.com/heini208/fault_detector_msgs) – custom message definitions for this system

- **Behaviour Trees**
  - [`py_trees`](https://github.com/splintered-reality/py_trees)
  - [`py_trees_ros`](https://github.com/splintered-reality/py_trees_ros)
  - `py_trees_ros_interfaces`

- **Perception**
  - [`apriltag_ros`](https://github.com/christianrauch/apriltag_ros) (or [AprilRobotics/apriltag_ros](https://github.com/AprilRobotics/apriltag_ros))
  - `pointcloud_to_laserscan` (if using the Nav2 + synthetic scan pipeline)

- **Mapping & Navigation**
  - [`rtabmap_ros`](https://github.com/introlab/rtabmap_ros) (and `rtabmap_slam`, `rtabmap_sync`)
  - [`nav2_bringup`](https://github.com/ros-planning/navigation2) and full Nav2 stack

- **UI & Tools**
  - `PyQt5` (Python package) – for GUI
  - `rviz2` – visualization

Python dependencies (partial):

```bash
pip install PyQt5 psutil
```

ROS dependencies are declared in [`package.xml`](package.xml); use `rosdep` to install what’s missing:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

---

## 3. Building the Package

Assuming a ROS 2 workspace `~/ros2_ws`:

```bash
cd ~/ros2_ws/src
git clone https://github.com/heini208/fault_detector_spot.git
git clone https://github.com/heini208/fault_detector_msgs.git
# plus spot_ros2, rtabmap_ros, nav2, etc., if not already in your workspace

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

Make sure the Spot SDK and `spot_ros2` setup instructions are followed as described in the [`spot_ros2` README](https://github.com/bdaiinstitute/spot_ros2).

---

## 4. Launching the System

### 4.1 Real robot (Fault Detector Spot stack)

The **primary launch file** is `fault_detector_launch.py`, which starts:

- `fault_detector_ui` – PyQt5 GUI
- `bt_runner` – main behaviour tree node
- `apriltag_node` – AprilTag detection for hand camera (`apriltag_ros`)
- `record_manager` – command recording & playback node

From your ROS 2 workspace:

```bash
source install/setup.bash

ros2 launch fault_detector_spot fault_detector_launch.py
```

Requirements:

- Spot is powered on and connected to the ROS machine (via `spot_ros2` configuration).
- `fault_detector_msgs` and `spot_ros2` are built and sourced.
- AprilTag config file (e.g. `config/my_tags.yaml`) matches your tags in the environment.

### 4.2 Simulation / reduced setup

For a simplified, simulation‑oriented setup:

```bash
ros2 launch fault_detector_spot sim_fault_detector_launch.py
```

This launches:

- `fault_detector_ui`
- `record_manager`
- `sim_bt_runner` (instead of the full `bt_runner`)

You are expected to provide simulated topics for the UI and BT (e.g. via Gazebo or your own nodes).

### 4.3 Mapping and Localization (RTAB‑Map)

RTAB‑Map is launched isolated via [`rtab_mapping_launch.py`](rtab_mapping_launch.py). This launch file:

- Synchronizes multiple RGB‑D streams using `rtabmap_sync/rgbd_sync`
- Starts `rtabmap_slam/rtabmap` in:

  - **mapping mode** (extend map) or
  - **localization‑only mode** (no map changes)

- Launches RViz with `config/mapping.rviz` for visualization

Example:

```bash
ros2 launch fault_detector_spot rtab_mapping_launch.py \
  db_path:=/path/to/your_map.db \
  delete_db:=false \
  extend_map:=true
```

See Section **10.5 Implementation Overview** and **10.6 Map lifecycle and process control** in [`System_Design.md`](System_Design.md) for the full flow.

### 4.4 Navigation (Nav2)

Nav2 is brought up isolated with [`nav2_spot_launch.py`](nav2_spot_launch.py). This:

- Includes `nav2_bringup/bringup_launch.py` with custom params
- Creates synthetic `/scan` topic from multiple depth cameras (`pointcloud_to_laserscan`)
- Starts the `nav2_cmd_vel_gate` node to coordinate Nav2 and Spot base control

Example:

```bash
ros2 launch fault_detector_spot nav2_spot_launch.py \
  use_sim_time:=false \
  map:=/path/to/your_map.yaml
```

The behaviour tree (`bt_runner`) interacts with Nav2 via the `NavigateToGoalPose` behaviour and Nav2’s `/navigate_to_pose` action.

---

## 5. Documentation Overview

If you want to **understand or extend** the system, follow this reading order:

1. **High‑level architecture & components**
   - [`System_Design.md`](System_Design.md)
     - Section 1 – General Architecture
     - Section 2 – Main System Components and Features
     - Section 3 – Data Flow Summary
2. **Interfaces & commands**
   - `System_Design.md` Section 5 – Interface Summary
   - `fault_detector_msgs` repo:
     - [`ComplexCommand.msg`](https://github.com/heini208/fault_detector_msgs/blob/main/msg/ComplexCommand.msg)
     - [`CommandRecordControl.msg`](https://github.com/heini208/fault_detector_msgs/blob/main/msg/CommandRecordControl.msg)
   - (Optional) `detailed_command_descriptions.md` in this repo, if present
3. **Behaviour Tree internals**
   - `System_Design.md` Sections 6–9
   - [`bt_runner.py`](fault_detector_spot/behaviour_tree/bt_runner.py)
4. **Mapping & Navigation**
   - `System_Design.md` Section 10
   - [`rtab_mapping_launch.py`](rtab_mapping_launch.py)
   - [`nav2_spot_launch.py`](nav2_spot_launch.py) and `config/nav2_spot_params.yaml`
5. **Recording & Playback, UI**
   - `System_Design.md` Sections 11–12
6. **Full list of external references**
   - `System_Design.md` Section 14 – References

---

## 7. Quick Start Checklist

1. **Hardware & network**
   - Spot online and reachable from your ROS machine
   - Spot time synchronized reasonably well with ROS machine (for TF/SLAM)

2. **Software**
   - ROS 2 Humble environment sourced
   - `spot_ros2` working (you can command Spot via its own examples)
   - `fault_detector_msgs` and `fault_detector_spot` built successfully
   - `rtabmap_ros`, `nav2` and `apriltag_ros` installed

3. **Bring up the stack**
   - Start RTAB‑Map (if you want mapping/localization)
   - Start Nav2 (if you want navigation to waypoints)
   - Start the Fault Detector stack:

     ```bash
     ros2 launch fault_detector_spot fault_detector_launch.py
     ```

4. **Use the UI**
   - Send simple commands (e.g. `STAND_UP`, `READY_ARM`)
   - Create a map and waypoints
   - Move between waypoints and to tags
   - Record and replay a sequence

For detailed behaviour descriptions and design rationale, always refer back to  
[`System_Design.md`](System_Design.md) and the [`fault_detector_msgs`](https://github.com/heini208/fault_detector_msgs) message definitions.
