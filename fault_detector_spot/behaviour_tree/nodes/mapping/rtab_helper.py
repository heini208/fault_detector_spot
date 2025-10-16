import json
import os
import signal
import subprocess
import time

import py_trees
from ament_index_python.packages import get_package_share_directory
from fault_detector_msgs.msg import StringArray
from fault_detector_spot.behaviour_tree.QOS_PROFILES import LATCHED_QOS
from fault_detector_spot.behaviour_tree.nodes.navigation.nav2_helper import Nav2Helper
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class RTABHelper:
    """
    Helper class for managing RTAB-Map SLAM and Localization processes.
    Integrates with py_trees Blackboard and ROS2 Node for publishing map state.
    """

    def __init__(self, node, blackboard, nav2_launch_file="nav2_sim_launch.py", nav2_params_file=None,
                 launch_file="rtab_mapping_launch.py"):

        self.node = node
        self.slam_launch_file = launch_file
        self.bb = blackboard
        self.maps_dir = os.path.join(get_package_share_directory("fault_detector_spot"), "maps")
        os.makedirs(self.maps_dir, exist_ok=True)

        # Ensure blackboard keys exist
        self.init_blackboard_keys()
        self.init_ros_publishers()

        self.nav2_helper = Nav2Helper(
            node=self.node,
            blackboard=self.bb,
            launch_file=nav2_launch_file,
            params_file=nav2_params_file
        )

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
        # ROS publishers
        self.active_map_pub = self.node.create_publisher(String, "active_map", LATCHED_QOS)
        self.map_list_pub = self.node.create_publisher(StringArray, "map_list", LATCHED_QOS)
        self.waypoint_list_pub = self.node.create_publisher(StringArray, "waypoint_list", LATCHED_QOS)
        self.landmark_list_pub = self.node.create_publisher(StringArray, "landmark_list", LATCHED_QOS)

    def _db_path(self, map_name: str):
        return os.path.join(self.maps_dir, f"{map_name}.db")

    def get_json_path(self, map_name: str):
        return os.path.join(self.maps_dir, f"{map_name}.json")

    def _publish_active_map(self):
        if self.bb.active_map_name:
            msg = String()
            msg.data = self.bb.active_map_name
            self.active_map_pub.publish(msg)
            self.publish_waypoint_list()
            self.publish_landmark_list()

    def update_map_list(self, extra_map: str = None):
        maps = [f[:-3] for f in os.listdir(self.maps_dir) if f.endswith(".db")]
        if extra_map and extra_map not in maps:
            maps.append(extra_map)
        msg = StringArray()
        msg.names = maps
        self.map_list_pub.publish(msg)

    def save_static_map(self, path: str) -> bool:
        """
        Save the static map using RTAB-Map's /rtabmap/save_map service.

        Args:
            path: Full path (without extension) where the map should be saved.

        Returns:
            True if successful, False otherwise.
        """
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self._call_service("/rtabmap/save_db")

    def stop_current_process(self):
        """
        Stop RTAB-Map and Nav2 safely.
        - If in mapping mode: pause RTAB-Map, publish final map.
        - Then stop the RTAB-Map and Nav2 processes.
        """
        proc = getattr(self.bb, "slam_launch_process", None)

        if proc:
            running_mode = self._get_running_mode()
            map_name = self.bb.active_map_name or "unnamed_map"
            # If we are in mapping mode → save before shutdown
            if running_mode == "mapping":
                self.node.get_logger().info(f"Pausing RTAB-Map before shutdown...")
                self._call_service("/rtabmap/pause")

                self.node.get_logger().info(f"Saving the RTAB-Map database to disk...")
                self._call_service("/rtabmap/save_db")

                self.node.get_logger().info(f"Publishing latest map to topics...")
                self._call_service("/rtabmap/publish_map")
                time.sleep(5)

            # Stop RTAB-Map process
            self.node.get_logger().info("Stopping RTAB-Map process...")
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=3)
            self.bb.slam_launch_process = None

        self.stop_nav2()

    def stop_without_save(self):
        """Stop Slam Toolbox and Nav2 without saving the map."""
        proc = getattr(self.bb, "slam_launch_process", None)
        if proc:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                proc.wait(timeout=2)
            except Exception as e:
                self.node.get_logger().warn(f"Failed to stop Slam Toolbox: {e}")
            finally:
                self.bb.slam_launch_process = None

        self.stop_nav2()

    def stop_nav2(self):
        if self.nav2_helper.is_running():
            self.node.get_logger().info("Stopping Nav2...")
            self.nav2_helper.stop()

    def _call_service(self, service_name: str, srv_type=None, request=None, timeout_sec=2.0):
        """
        Call a ROS2 service and wait for it to complete.
        - service_name: full service name, e.g., "/rtabmap/pause"
        - srv_type: service type, e.g., std_srvs.srv.Empty (optional if Empty service)
        - request: optional request object
        - timeout_sec: max time to wait for service response
        """
        if srv_type is None:
            from std_srvs.srv import Empty
            srv_type = Empty

        client = self.node.create_client(srv_type, service_name)
        if not client.wait_for_service(timeout_sec=timeout_sec):
            print(f"Service {service_name} not available")
            return False

        if request is None:
            request = srv_type.Request()

        future = client.call_async(request)
        if not future:
            return False

        start_time = time.time()
        while not future.done():
            time.sleep(0.05)  # tiny sleep so the BT tick can continue elsewhere
            if time.time() - start_time > timeout_sec:
                self.node.get_logger().warn(f"Service {service_name} timed out after {timeout_sec}s")
                return False

        return True

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

    def start_mapping_from_existing(self, map_name: str = None, extend_map: bool = True, delete_db: bool = False,
                                    rviz: bool = True):
        if not self.is_rtabmap_running():
            return self.initialize_mapping_from_existing(map_name, extend_map, delete_db, rviz)
        else:
            if self.nav2_helper.is_running():
                self.nav2_helper.stop()
            if extend_map:
                self.set_mode_mapping()
            else:
                self.set_mode_localization()
            return self.bb.slam_launch_process

    def initialize_mapping_from_existing(self, map_name: str = None, extend_map: bool = True, delete_db: bool = False,
                                         rviz: bool = True):
        """
                Start RTAB-Map in mapping or localization mode.

                Args:
                    map_name: Name of the map (used to locate the .db file).
                    extend_map: True for mapping mode, False for localization mode.
                    delete_db: Whether to delete the database on start.
                    rviz: Whether to launch RViz.
                """
        # Stop any running mapping/localization process
        self.stop_current_process()

        # Determine map name
        if map_name is None:
            map_name = self.bb.active_map_name
            if map_name is None:
                raise RuntimeError("No active map set to start mapping/localization")

        # Build DB path
        db_path = self._db_path(map_name)

        # Ensure directory exists
        os.makedirs(os.path.dirname(db_path), exist_ok=True)

        # Convert booleans to lowercase strings
        extend_map_str = "true" if extend_map else "false"
        delete_db_str = "true" if delete_db else "false"
        rviz_str = "true" if rviz else "false"

        # Build the ros2 launch command
        args = [
            "ros2", "launch", "fault_detector_spot", self.slam_launch_file,
            f"db_path:={db_path}",
            f"delete_db:={delete_db_str}",
            f"extend_map:={extend_map_str}",
            f"rviz:={rviz_str}"
        ]

        # Start the launch process
        proc = subprocess.Popen(args, preexec_fn=os.setsid)

        # Store process info and update state
        self.bb.slam_launch_process = proc
        self.bb.active_map_name = map_name
        self._publish_active_map()

        self.node.get_logger().info(
            f"Started RTAB-Map in {'mapping' if extend_map else 'localization'} mode with DB: {db_path}"
        )

        return proc

    def start_mapping_from_scratch(self, map_name: str = None):
        """
        Start RTAB-Map mapping completely from scratch (deletes any existing map/database).

        Creates a new JSON waypoint/landmark file if needed and launches RTAB-Map
        in mapping mode with a new database file.
        """
        self.stop_current_process()

        # Determine the map name
        if map_name is None:
            map_name = self.bb.active_map_name
            if map_name is None:
                raise RuntimeError("No active map name provided to start mapping from scratch")

        # Update internal map list and ensure paths exist
        self.update_map_list(map_name)
        json_path = os.path.join(self.maps_dir, f"{map_name}.json")

        # Create an empty JSON file if not existing
        if not os.path.exists(json_path):
            data = {"waypoints": [], "landmarks": []}
            os.makedirs(os.path.dirname(json_path), exist_ok=True)
            with open(json_path, "w") as f:
                json.dump(data, f, indent=4)
            self.node.get_logger().info(f"Created new JSON map file: {json_path}")

        # Launch RTAB-Map in fresh mapping mode
        self.node.get_logger().info(f"Starting a new RTAB-Map mapping session for '{map_name}'")
        return self.initialize_mapping_from_existing(map_name=map_name, extend_map=True, delete_db=True, rviz=True)

    def start_localization(self, map_name: str = None, rviz: bool = True):
        if not self.is_rtabmap_running():
            self.init_localization(map_name, rviz)
        else:
            self.set_mode_localization()

        if not self.nav2_helper.is_running():
            self.nav2_helper.start()

        return self.bb.slam_launch_process

    def init_localization(self, map_name: str = None, rviz: bool = True):
        """
                Start RTAB-Map in localization-only mode using an existing database.
                """
        self.stop_current_process()

        # Determine which map to use
        if map_name is None:
            map_name = self.bb.active_map_name
            if map_name is None:
                raise RuntimeError("No active map specified to start localization")

        db_path = self._db_path(map_name)
        if not os.path.exists(db_path):
            raise FileNotFoundError(f"Database file not found for map: {db_path}")

        # Ensure directory exists
        os.makedirs(os.path.dirname(db_path), exist_ok=True)

        # Convert RViz flag to string for the launch argument
        rviz_str = "true" if rviz else "false"

        # Launch RTAB-Map in localization mode (extend_map=False)
        args = [
            "ros2", "launch", "fault_detector_spot", self.slam_launch_file,
            f"db_path:={db_path}",
            "delete_db:=false",
            "extend_map:=false",
            f"rviz:={rviz_str}"
        ]

        proc = subprocess.Popen(args, preexec_fn=os.setsid)

        # Save state
        self.bb.slam_launch_process = proc
        self.bb.active_map_name = map_name
        self._publish_active_map()

        self.node.get_logger().info(f"Started RTAB-Map localization with database: {db_path}")
        return proc

    def set_mode_localization(self):
        """Switch RTAB-Map to localization mode via service."""
        self._call_service("/rtabmap/set_mode_localization")

    def set_mode_mapping(self):
        """Switch RTAB-Map to mapping mode via service."""
        self._call_service("/rtabmap/set_mode_mapping")

    def is_rtabmap_running(self) -> bool:
        proc = self.bb.slam_launch_process
        if proc is None:
            return False
        return proc.poll() is None

    def _get_running_mode(self) -> str:
        if not self.is_rtabmap_running():
            return "none"
        result = subprocess.run(
            ["ros2", "param", "get", "/rtabmap", "localization"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
        if "True" in result.stdout:
            return "localization"
        else:
            return "mapping"

    def is_slam_running(self) -> bool:
        """Check if RTABMAPPING is currently running."""
        return self._get_running_mode() == "mapping"

    def change_map(self, map_name: str):
        """
        Switch RTAB-Map to a different map while preserving the current mode
        (mapping or localization). If RTAB-Map is not running, only updates
        the active map metadata.
        """
        if not map_name:
            raise RuntimeError("No map specified to switch to")
        if map_name == self.bb.active_map_name:
            self.node.get_logger().info(f"Map '{map_name}' already active")
            return True
        current_mode = self._get_running_mode()
        if current_mode == "none":
            self.bb.active_map_name = map_name
            self._publish_active_map()
            self.node.get_logger().info(f"Set active map to '{map_name}' (no running process).")
            return True

        # Stop current RTAB-Map process
        self.stop_current_process()
        self.node.get_logger().info(f"Switching to map '{map_name}' in {current_mode} mode...")

        # Restart RTAB-Map in the same mode but with a new database
        if current_mode == "mapping":
            proc = self.initialize_mapping_from_existing(map_name)
        else:  # localization
            proc = self.start_localization(map_name)
        self.feedback_message = f"Changed to map '{map_name}' in {current_mode} mode"
        self.node.get_logger().info(self.feedback_message)
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

    def delete_landmark(self, landmark_name: str, map_name: str = None):
        map_name = map_name or self.bb.active_map_name
        if not map_name:
            raise ValueError("No active map set")

        json_path = self.get_json_path(map_name)
        if not os.path.exists(json_path):
            return False

        with open(json_path, "r") as f:
            data = json.load(f)

        # Filter out the landmark with the given name
        new_landmarks = [lm for lm in data.get("landmarks", []) if lm["name"] != landmark_name]
        if len(new_landmarks) == len(data.get("landmarks", [])):
            # Nothing was removed
            return False

        data["landmarks"] = new_landmarks

        # Save updated JSON
        with open(json_path, "w") as f:
            json.dump(data, f, indent=2)

        # Publish updated list if it's the active map
        if map_name == self.bb.active_map_name:
            self.publish_landmark_list(map_name)  # implement similar to publish_waypoint_list
        return True