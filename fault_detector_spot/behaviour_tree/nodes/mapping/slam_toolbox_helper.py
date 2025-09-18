import json
import math
import os
import signal
import subprocess

import py_trees
import rclpy
import tf2_ros
from ament_index_python.packages import get_package_share_directory
from fault_detector_msgs.msg import StringArray
from fault_detector_spot.behaviour_tree.QOS_PROFILES import LATCHED_QOS
from fault_detector_spot.behaviour_tree.nodes.navigation.nav2_helper import Nav2Helper
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.time import Time
from slam_toolbox.srv import DeserializePoseGraph
from std_msgs.msg import String
from tf2_geometry_msgs import tf2_geometry_msgs


class SlamToolboxHelper():
    """
    Helper class for managing Slam Toolbox mapping/localization and Nav2 processes.
    Uses .posegraph for map storage instead of RTAB-Map .db.
    """

    def __init__(self, node, blackboard, nav2_launch_file="nav2_sim_launch.py", nav2_params_file=None,
                 launch_file="slam_sim_merged_launch.py"):
        self.node = node
        self.slam_launch_file = launch_file
        self.bb = blackboard
        self.recordings_dir = os.path.join(
            get_package_share_directory("fault_detector_spot"), "maps"
        )

        # Blackboard setup
        self.init_blackboard_keys()
        self.init_ros_publishers()
        self.init_pose_subscriber()

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

    def init_pose_subscriber(self):
        """
        Subscribe to Slam Toolbox's /pose topic for current localization.
        """
        self.slam_pose = None
        self.slam_pose_sub = self.node.create_subscription(
            PoseWithCovarianceStamped,
            "/pose",  # Slam Toolbox publishes PoseStamped here
            self._slam_pose_callback,
            10
        )

    def _slam_pose_callback(self, msg: PoseWithCovarianceStamped):
        self.slam_pose = msg

    def get_slam_pose_as_pose_stamped(self) -> PoseStamped | None:
        """
        Returns a copy of the last Slam Toolbox localization pose as PoseStamped.
        If no pose has been received yet, returns None.
        """
        if self.slam_pose is None or self.slam_pose.pose is None:
            return None

        pose_copy = PoseStamped()
        pose_copy.header = self.slam_pose.header
        pose_copy.pose = self.slam_pose.pose.pose
        return pose_copy

    def init_blackboard_keys(self):
        self.bb.register_key("active_map_name", access=py_trees.common.Access.WRITE)
        self.bb.register_key("slam_launch_process", access=py_trees.common.Access.WRITE)
        self.bb.register_key("nav2_launch_process", access=py_trees.common.Access.WRITE)
        self.bb.register_key("last_pose_estimation", access=py_trees.common.Access.READ)

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
        self.landmark_list_pub = self.node.create_publisher(StringArray, "landmark_list", LATCHED_QOS)

    def _posegraph_path(self, map_name: str):
        return os.path.join(self.recordings_dir, f"{map_name}.posegraph")

    def _posegraph_without_ext(self, map_name: str):
        return os.path.join(self.recordings_dir, f"{map_name}")

    def get_json_path(self, map_name: str):
        return os.path.join(self.recordings_dir, f"{map_name}.json")

    def _publish_active_map(self):
        if self.bb.active_map_name:
            msg = String()
            msg.data = self.bb.active_map_name
            self.active_map_pub.publish(msg)
            self.publish_waypoint_list()
            self.publish_landmark_list()

    def update_map_list(self, extra_map: str = None):
        maps = [f[:-10] for f in os.listdir(self.recordings_dir) if f.endswith(".posegraph")]
        if extra_map and extra_map not in maps:
            maps.append(extra_map)
        msg = StringArray()
        msg.names = maps
        self.map_list_pub.publish(msg)

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

    def get_last_localization_pose_xytheta(self) -> list[float]:
        """
        Get the last localization pose as [x, y, theta].
        Defaults to [0, 0, 0] if no pose is available.
        """
        pose_stamped: PoseStamped | None = self.get_last_localization_pose()
        if pose_stamped is None:
            return [0.0, 0.0, 0.0]

        x = pose_stamped.pose.position.x
        y = pose_stamped.pose.position.y

        q = pose_stamped.pose.orientation
        # Convert quaternion to yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        return [x, y, theta]

    def get_odom_pose_stamped(self) -> PoseStamped | None:
        """
        Returns the current odom pose as a PoseStamped (map frame 'odom').
        """
        try:
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
        except Exception as e:
            self.node.get_logger().warn(f"Failed to get odom -> base_link transform: {e}")
            return None

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
        pose_stamped.header.frame_id = 'odom'

        pose_stamped.pose.position.x = trans.transform.translation.x
        pose_stamped.pose.position.y = trans.transform.translation.y
        pose_stamped.pose.position.z = trans.transform.translation.z

        # Convert quaternion from transform
        pose_stamped.pose.orientation = trans.transform.rotation

        return pose_stamped

    def get_last_localization_pose(self, tolerance_sec: float = 0.1) -> PoseStamped | None:
        """
        Return the most recent localization pose.
        Prioritize AMCL if it is newer than Slam Toolbox pose within a tolerance.
        """
        pose = PoseStamped()
        last_pose = self.bb.last_pose_estimation
        pose.header = last_pose.header
        pose.pose = last_pose.pose.pose

        return pose

    def _launch_slam_toolbox(self, map_name: str, mode: str = "mapping"):
        """Launch Slam Toolbox with specified map and mode."""
        map_start_pose = self.get_last_localization_pose_xytheta()  # [x, y, theta]
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
            data = {
                "waypoints": [],
                "landmarks": []
            }
            with open(json_path, "w") as f:
                json.dump(data, f, indent=4)

        return self._launch_slam_toolbox(map_name, mode="mapping")

    def start_mapping_from_existing(self, map_name: str = None):
        """Start mapping, continuing from a previously saved posegraph."""
        if map_name is None:
            map_name = self.bb.active_map_name
        self.stop_current_process()
        proc = self._launch_slam_toolbox(map_name, mode="mapping")
        # self._deserialize_map(map_name)
        return proc

    def start_localization(self, map_name: str = None):
        """
        Start Slam Toolbox in localization mode using a previously saved serialized map.
        Uses the current odom pose as map_start_pose.
        """
        if map_name is None:
            map_name = self.bb.active_map_name
        self.stop_current_process()
        map_path_no_ext = self._posegraph_without_ext(map_name)
        proc = self.nav2_helper.start(map_file=map_path_no_ext + ".yaml", initial_pose = self.get_last_localization_pose_xytheta())

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
        if map_name == self.bb.active_map_name:
            self.node.get_logger().info(f"Map '{map_name}' already active")
            return True
        # always update active_map_name and topics
        mode = self._get_slam_mode()
        if mode == "none":
            self.node.get_logger().info(f"Slam Toolbox not running; active map set to '{map_name}'")
            self.bb.active_map_name = map_name
            self._publish_active_map()
            return True
        elif mode == "mapping":
            self.start_mapping_from_existing(map_name)
        elif mode == "localization":
            self.start_localization(map_name)
        else:
            raise RuntimeError(f"Unknown current mode '{mode}'")

        self.node.get_logger().info(f"Changed to map '{map_name}' in {mode} mode")
        return True

    # --- Waypoint Management ---
    def publish_waypoint_list(self, map_name: str = None):
        names = self.get_list_from_json_category("waypoints", map_name)
        msg = StringArray()
        msg.names = names
        self.waypoint_list_pub.publish(msg)

    def publish_landmark_list(self, map_name: str = None):
        names = self.get_list_from_json_category("landmarks", map_name)
        msg = StringArray()
        msg.names = names
        self.landmark_list_pub.publish(msg)

    def get_list_from_json_category(self, category: String, map_name: str = None, ):
        map_name = map_name or self.bb.active_map_name
        if not map_name:
            return
        json_path = self.get_json_path(map_name)
        if not os.path.exists(json_path):
            return
        with open(json_path, "r") as f:
            data = json.load(f)
        names = [wp["name"] for wp in data.get(category, [])]
        return names

    def add_pose_as_waypoint(self, waypoint_name: str, pose: PoseStamped, map_name: str = None):
        self.add_pose_to_json("waypoints", waypoint_name, pose, map_name)

        if map_name == self.bb.active_map_name:
            self.publish_waypoint_list(map_name)

    def add_pose_as_landmark(self, landmark_name: str, pose: PoseStamped, map_name: str = None):
        self.add_pose_to_json("landmarks", landmark_name, pose, map_name)

        if map_name == self.bb.active_map_name:
            self.publish_landmark_list(map_name)

    def add_pose_to_json(self, category: String, entry_name: String, pose: PoseStamped, map_name: str = None):
        map_name = map_name or self.bb.active_map_name
        if not map_name:
            raise ValueError("No active map set")

        json_path = self.get_json_path(map_name)
        if os.path.exists(json_path):
            with open(json_path, "r") as f:
                data = json.load(f)
        else:
            data = {category: []}

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
        for wp in data[category]:
            if wp["name"] == entry_name:
                wp["pose"] = pose_dict
                updated = True
                break
        if not updated:
            data[category].append({"name": entry_name, "pose": pose_dict})

        with open(json_path, "w") as f:
            json.dump(data, f, indent=2)

    def delete_waypoint(self, waypoint_name: str, map_name: str = None):
        map_name = map_name or self.bb.active_map_name
        if not map_name:
            raise ValueError("No active map set")

        json_path = self.get_json_path(map_name)
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

    def transform_to_map_frame(self, pose: PoseStamped, source_frame: str) -> PoseStamped | None:
        """
        Transform a pose from a source frame into the map frame.

        Args:
            pose: PoseStamped in the source frame.
            source_frame: The frame the pose is expressed in.

        Returns:
            PoseStamped in map frame, or None if the transform fails.
        """
        print(self.tf_buffer.all_frames_as_yaml())

        stamp = Time.from_msg(pose.header.stamp)
        trans = self.tf_buffer.lookup_transform(
            "map",
            source_frame,
            stamp,
            rclpy.duration.Duration(seconds=5.0)
        )

        transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, trans)
        transformed_pose.header.frame_id = "map"
        return transformed_pose