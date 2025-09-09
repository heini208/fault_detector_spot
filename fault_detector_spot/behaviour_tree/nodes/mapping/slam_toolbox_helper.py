import math
import os
import signal
import subprocess
import json

import py_trees
import rclpy
import tf2_ros
from rclpy import Future
from slam_toolbox.srv import DeserializePoseGraph, SaveMap
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose
from fault_detector_msgs.msg import StringArray
from fault_detector_spot.behaviour_tree.nodes.navigation.nav2_helper import Nav2Helper
from fault_detector_spot.behaviour_tree.QOS_PROFILES import LATCHED_QOS
from ament_index_python.packages import get_package_share_directory


class SlamToolboxHelper:
    """
    Helper class for managing Slam Toolbox mapping/localization and Nav2 processes.
    Uses .posegraph for map storage instead of RTAB-Map .db.
    """

    def __init__(self, node, blackboard, nav2_launch_file="nav2_sim_launch.py", nav2_params_file=None, launch_file="slam_sim_merged_launch.py"):
        self.node = node
        self.slam_launch_file = launch_file
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

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

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
        return os.path.join(self.recordings_dir, f"{map_name}.posegraph")

    def _posegraph_without_ext(self, map_name: str):
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

    import os
    import subprocess

    def save_static_map(self, path: str) -> bool:
        """
        Save the static occupancy grid map using `map_saver_cli`.

        Args:
            path: Full path (without extension) where map should be saved.

        Returns:
            True if the map was successfully saved, False otherwise.
        """
        # Make sure the directory exists
        os.makedirs(os.path.dirname(path), exist_ok=True)

        # Build the command
        cmd = [
            "ros2", "run", "nav2_map_server", "map_saver_cli",
            "-f", path,
            "--ros-args", "-p", "map_subscribe_transient_local:=true"
        ]

        try:
            # Run the command and wait for it to finish
            result = subprocess.run(cmd, check=True, capture_output=True, text=True)
            self.node.get_logger().info(f"Map successfully saved to {path}.yaml")
            return True
        except subprocess.CalledProcessError as e:
            self.node.get_logger().error(
                f"Failed to save map: {e.stderr.strip()}"
            )
            return False

    def stop_current_process(self):
        """Stop Slam Toolbox and Nav2 safely, serialize map if in mapping mode."""
        proc = getattr(self.bb, "slam_launch_process", None)
        if proc:
            try:
                # Determine if we are in mapping mode
                    # Serialize current map before stopping
                posegraph_path = self._posegraph_without_ext(self.bb.active_map_name)
                self.node.get_logger().info(f"Serializing posegraph: {posegraph_path}")
                os.system(
                    f"ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph '{{filename: \"{posegraph_path}\"}}'")

                self.save_static_map(posegraph_path)

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


    def get_odom_pose(self) -> list[float] | None:
        """
        Returns the current odom pose as [x, y, theta] for Slam Toolbox localization.
        """
        try:
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
        except Exception as e:
            self.node.get_logger().warn(f"Failed to get odom -> base_link transform: {e}")
            return None

        x = trans.transform.translation.x
        y = trans.transform.translation.y

        # Convert quaternion to yaw (theta)
        q = trans.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        return [x, y, theta]

    def _launch_slam_toolbox(self, map_name: str, mode: str = "mapping"):
        """Launch Slam Toolbox with specified map and mode."""
        map_start_pose = self.get_odom_pose()  # [x, y, theta]
        if map_start_pose is None:
            map_start_pose = [0.0, 0.0, 0.0]

        # Convert to string for launch argument
        pose_str = ",".join([f"{v:.6f}" for v in map_start_pose])

        path = self._posegraph_without_ext(map_name)
        args = [
            "ros2", "launch", "fault_detector_spot", self.slam_launch_file,
            f"map_db_path:={path}",
            f"mode:={mode}",
            "launch_rviz:=true",
            f"map_start_pose:={pose_str}"
        ]
        proc = subprocess.Popen(args, preexec_fn=os.setsid)
        self.bb.slam_launch_process = proc
        self.bb.active_map_name = map_name
        self._publish_active_map()
        return proc

    def _deserialize_map(self, map_name: str):
        """Call Slam Toolbox service to load a saved posegraph."""
        cli = self.node.create_client(DeserializePoseGraph, '/slam_toolbox/deserialize_map')
        while not cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Waiting for /slam_toolbox/deserialize_map service...")
        req = DeserializePoseGraph.Request()
        req.filename = self._posegraph_path(map_name)
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        self.node.get_logger().info(f"Deserialized map: {map_name}")

    def start_mapping_from_scratch(self, map_name: str = None):
        """Start a brand-new map from scratch."""
        self.stop_current_process()
        self.update_map_list(map_name)
        json_path = os.path.join(self.recordings_dir, f"{map_name}.json")

        # Create empty JSON for waypoints if it does not exist
        if not os.path.exists(json_path):
            import json
            with open(json_path, "w") as f:
                json.dump({"waypoints": []}, f, indent=4)

        return self._launch_slam_toolbox(map_name, mode="mapping")

    def start_mapping_from_existing(self, map_name: str = None):
        """Start mapping, continuing from a previously saved posegraph."""
        if map_name is None:
            map_name = self.bb.active_map_name
        self.stop_current_process()
        proc = self._launch_slam_toolbox(map_name, mode="mapping")
        #self._deserialize_map(map_name)
        return proc

    def start_localization(self, map_name: str = None, slam_launch="slam_toolbox_launch.py"):
        """
        Start Slam Toolbox in localization mode using a previously saved serialized map.
        Uses the current odom pose as map_start_pose.
        """
        if map_name is None:
            map_name = self.bb.active_map_name
        self.stop_current_process()
        map_path_no_ext = self._posegraph_without_ext(map_name)
        proc = self.nav2_helper.start(map_file=map_path_no_ext + ".yaml", initial_pose=self.get_odom_pose())

        return proc


    def is_slam_running(self) -> bool:
        """Check if Slam Toolbox is currently running."""
        proc = self.bb.slam_launch_process
        return proc is not None and proc.poll() is None

    def _get_slam_mode(self) -> str:
        if self.is_slam_running():
            return "mapping"
        elif self.nav2_helper.is_running():
            return "localization"
        else:
            return "none"


    def change_map(self, map_name: str):
        """
        Change the active map. If Slam Toolbox is running, restart in current mode
        with the new map. If not running, just update active_map_name.
        """
        if not map_name:
            raise RuntimeError("No map name provided for change_map")

        # always update active_map_name and topics
        mode = self._get_slam_mode()
        if mode == "none":
            self.node.get_logger().info(f"Slam Toolbox not running; active map set to '{map_name}'")
            self.update_map_list(map_name)
            return True
        elif mode == "mapping":
            self.start_mapping_from_existing(map_name)
        elif mode == "localization":
            self.start_localization(map_name)
        else:
            raise RuntimeError(f"Unknown current mode '{mode}'")

        self.node.get_logger().info(f"Changed to map '{map_name}' in {mode} mode")
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
