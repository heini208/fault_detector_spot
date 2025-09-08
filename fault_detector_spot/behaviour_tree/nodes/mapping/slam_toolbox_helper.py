import os
import signal
import subprocess
import json

import py_trees
import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from fault_detector_msgs.msg import StringArray
from fault_detector_spot.behaviour_tree.nodes.navigation.nav2_helper import Nav2Helper
from fault_detector_spot.behaviour_tree.QOS_PROFILES import LATCHED_QOS
from ament_index_python.packages import get_package_share_directory


class SlamToolboxHelper:
    """
    Helper class for managing Slam Toolbox mapping/localization and Nav2 processes.
    Uses .posegraph for map storage instead of RTAB-Map .db.
    """

    def __init__(self, node, blackboard, nav2_launch_file="nav2_sim_launch.py", nav2_params_file=None):
        self.node = node
        self.bb = blackboard
        self.recordings_dir = os.path.join(
            get_package_share_directory("fault_detector_spot"), "maps"
        )

        # Blackboard setup
        self.init_blackboard_keys()
        self.init_ros_publishers()

        self.nav2_helper = Nav2Helper(
            node=self.node,
            blackboard=self.bb,
            launch_file=nav2_launch_file,
            params_file=nav2_params_file
        )

        # Load available maps
        self.update_map_list()

    def init_blackboard_keys(self):
        self.bb.register_key("active_map_name", access=py_trees.common.Access.WRITE)
        self.bb.register_key("slam_launch_process", access=py_trees.common.Access.WRITE)
        self.bb.register_key("nav2_launch_process", access=py_trees.common.Access.WRITE)

        if not self.bb.exists("active_map_name"):
            self.bb.active_map_name = None
        if not self.bb.exists("slam_launch_process"):
            self.bb.slam_launch_process = None
        if not self.bb.exists("nav2_launch_process"):
            self.bb.nav2_launch_process = None

    def init_ros_publishers(self):
        self.active_map_pub = self.node.create_publisher(String, "active_map", LATCHED_QOS)
        self.map_list_pub = self.node.create_publisher(StringArray, "map_list", LATCHED_QOS)
        self.waypoint_list_pub = self.node.create_publisher(StringArray, "waypoint_list", LATCHED_QOS)

    def _posegraph_path(self, map_name: str):
        return os.path.join(self.recordings_dir, f"{map_name}")

    def _json_path(self, map_name: str):
        return os.path.join(self.recordings_dir, f"{map_name}.json")

    def _publish_active_map(self):
        if self.bb.active_map_name:
            msg = String()
            msg.data = self.bb.active_map_name
            self.active_map_pub.publish(msg)
            self.publish_waypoint_list()

    def update_map_list(self, extra_map: str = None):
        maps = [f[:-10] for f in os.listdir(self.recordings_dir) if f.endswith(".posegraph")]
        if extra_map and extra_map not in maps:
            maps.append(extra_map)
        msg = StringArray()
        msg.names = maps
        self.map_list_pub.publish(msg)

    def stop_current_process(self):
        """Stop Slam Toolbox and Nav2 safely, serialize map if in mapping mode."""
        proc = getattr(self.bb, "slam_launch_process", None)
        if proc:
            try:
                # Determine if we are in mapping mode
                if getattr(self.bb, "active_map_name", None):
                    # Serialize current map before stopping
                    posegraph_path = self._posegraph_path(self.bb.active_map_name)
                    self.node.get_logger().info(f"Serializing posegraph: {posegraph_path}")
                    os.system(
                        f"ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph '{{filename: \"{posegraph_path}\"}}'")

                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                proc.wait(timeout=1)
            except Exception as e:
                self.node.get_logger().warn(f"Failed to stop Slam Toolbox: {e}")
            finally:
                self.bb.slam_launch_process = None

        if self.nav2_helper.is_running():
            self.nav2_helper.stop()

    def stop_without_save(self):
        """Stop Slam Toolbox and Nav2 without saving the map."""
        proc = getattr(self.bb, "slam_launch_process", None)
        if proc:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                proc.wait(timeout=1)
            except Exception as e:
                self.node.get_logger().warn(f"Failed to stop Slam Toolbox: {e}")
            finally:
                self.bb.slam_launch_process = None

        if self.nav2_helper.is_running():
            self.nav2_helper.stop()

    def start_mapping(self, map_name: str =  None, launch_file="slam_toolbox_launch.py", initial_pose: PoseStamped = None):
        """Start mapping, optionally continue on existing posegraph."""
        self.stop_current_process()
        if map_name is None:
            map_name = self.bb.active_map_name
            if map_name is None:
                raise RuntimeError("No active map set to start mapping")

        os.makedirs(self.recordings_dir, exist_ok=True)
        posegraph_path = self._posegraph_path(map_name)
        json_path = self._json_path(map_name)

        # Ensure waypoint file exists
        if not os.path.exists(json_path):
            with open(json_path, "w") as f:
                json.dump({"waypoints": []}, f, indent=2)

        # Deserialize map if exists
        if os.path.exists(posegraph_path):
            self.node.get_logger().info(f"Continuing existing map: {map_name}")
            os.system(
                f"ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph '{{filename: \"{posegraph_path}\"}}'")

        # Set initial pose for AMCL if provided
        if initial_pose:
            self.node.get_logger().info(f"Setting initial pose for mapping from localization")
            self.node.get_logger().info(f"{initial_pose}")
            # Publish to /initialpose topic so slam_toolbox continues correctly
            pub = self.node.create_publisher(PoseStamped, '/initialpose', 1)
            pub.publish(initial_pose)

        args = [
            "ros2", "launch", "fault_detector_spot", launch_file,
            f"map_db_path:={posegraph_path}",
            "mode:=mapping",
            "launch_rviz:=true"
        ]
        proc = subprocess.Popen(args, preexec_fn=os.setsid)
        self.bb.slam_launch_process = proc
        self.bb.active_map_name = map_name
        self._publish_active_map()
        self.update_map_list(map_name)
        return proc

    def start_localization(self, map_name: str = None, launch_file="slam_toolbox_launch.py"):
        """Switch to localization mode, start Nav2."""
        self.stop_current_process()
        if map_name is None:
            map_name = self.bb.active_map_name
            if map_name is None:
                raise RuntimeError("No active map set to start localization")

        posegraph_path = self._posegraph_path(map_name)

        if not os.path.exists(posegraph_path):
            raise RuntimeError(f"No posegraph file found for map '{map_name}'")

        args = [
            "ros2", "launch", "fault_detector_spot", launch_file,
            f"map_db_path:={posegraph_path}",
            "mode:=localization",
            "launch_rviz:=true"
        ]
        proc = subprocess.Popen(args, preexec_fn=os.setsid)
        self.bb.slam_launch_process = proc
        self.bb.active_map_name = map_name
        self._publish_active_map()

        if not self.nav2_helper.is_running():
            self.nav2_helper.start()

        return proc

    def is_slam_running(self) -> bool:
        """Check if Slam Toolbox is currently running."""
        proc = self.bb.slam_launch_process
        return proc is not None and proc.poll() is None

    def _get_slam_mode(self) -> str:
        """
        Returns current Slam Toolbox mode: "mapping" or "localization".
        Defaults to "mapping" if unable to detect.
        """
        try:
            result = subprocess.run(
                ["ros2", "param", "get", "/slam_toolbox", "mode"],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
            )
            if "localization" in result.stdout.lower():
                return "localization"
            return "mapping"
        except Exception:
            return "mapping"

    def change_map(self, map_name: str, slam_launch="slam_toolbox_launch.py"):
        """
        Change the active map. If Slam Toolbox is running, restart in current mode
        with the new map. If not running, just update active_map_name.
        """
        if not map_name:
            raise RuntimeError("No map name provided for change_map")

        self.bb.active_map_name = map_name
        self._publish_active_map()  # always update active_map_name and topics

        if not self.is_slam_running():
            # Slam Toolbox is off, no need to start anything
            self.node.get_logger().info(f"Slam Toolbox not running; active map set to '{map_name}'")
            self.update_map_list(map_name)
            return True

        # Slam Toolbox is running, restart it with new map in current mode
        self.stop_current_process()
        current_mode = self._get_slam_mode()
        if current_mode == "mapping":
            self.start_mapping(map_name, slam_launch)
        elif current_mode == "localization":
            self.start_localization(map_name, slam_launch)
        else:
            raise RuntimeError(f"Unknown current mode '{current_mode}'")

        self.node.get_logger().info(f"Changed to map '{map_name}' in {current_mode} mode")
        self.update_map_list(map_name)
        return True

    # --- Waypoint Management ---
    def publish_waypoint_list(self, map_name: str = None):
        map_name = map_name or self.bb.active_map_name
        if not map_name:
            return
        json_path = self._json_path(map_name)
        if not os.path.exists(json_path):
            return
        with open(json_path, "r") as f:
            data = json.load(f)
        names = [wp["name"] for wp in data.get("waypoints", [])]
        msg = StringArray()
        msg.names = names
        self.waypoint_list_pub.publish(msg)

    def add_pose_as_waypoint(self, waypoint_name: str, pose: PoseStamped, map_name: str = None):
        map_name = map_name or self.bb.active_map_name
        if not map_name:
            raise ValueError("No active map set")

        json_path = self._json_path(map_name)
        if os.path.exists(json_path):
            with open(json_path, "r") as f:
                data = json.load(f)
        else:
            data = {"waypoints": []}

        pose_dict = {
            "position": {
                "x": pose.pose.position.x,
                "y": pose.pose.position.y,
                "z": pose.pose.position.z,
            },
            "orientation": {
                "x": pose.pose.orientation.x,
                "y": pose.pose.orientation.y,
                "z": pose.pose.orientation.z,
                "w": pose.pose.orientation.w,
            },
        }

        updated = False
        for wp in data["waypoints"]:
            if wp["name"] == waypoint_name:
                wp["pose"] = pose_dict
                updated = True
                break
        if not updated:
            data["waypoints"].append({"name": waypoint_name, "pose": pose_dict})

        with open(json_path, "w") as f:
            json.dump(data, f, indent=2)

        if map_name == self.bb.active_map_name:
            self.publish_waypoint_list(map_name)

    def delete_waypoint(self, waypoint_name: str, map_name: str = None):
        map_name = map_name or self.bb.active_map_name
        if not map_name:
            raise ValueError("No active map set")

        json_path = self._json_path(map_name)
        if not os.path.exists(json_path):
            return False

        with open(json_path, "r") as f:
            data = json.load(f)

        new_wps = [wp for wp in data.get("waypoints", []) if wp["name"] != waypoint_name]
        if len(new_wps) == len(data.get("waypoints", [])):
            return False

        data["waypoints"] = new_wps
        with open(json_path, "w") as f:
            json.dump(data, f, indent=2)

        if map_name == self.bb.active_map_name:
            self.publish_waypoint_list(map_name)
        return True
