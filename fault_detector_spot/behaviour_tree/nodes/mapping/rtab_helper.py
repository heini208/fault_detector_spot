import json
import os
import signal
import subprocess

from fault_detector_msgs.msg import StringArray
from fault_detector_spot.behaviour_tree.nodes.navigation.nav2_helper import Nav2Helper
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import py_trees
from fault_detector_spot.behaviour_tree.QOS_PROFILES import LATCHED_QOS
import rclpy


class RTABHelper:
    """
    Helper class for managing RTAB-Map SLAM and Localization processes.
    Integrates with py_trees Blackboard and ROS2 Node for publishing map state.
    """

    def __init__(self, node, blackboard, nav2_launch_file="nav2_sim_launch.py", nav2_params_file=None):
        self.node = node
        self.bb = blackboard
        self.recordings_dir = os.path.join(get_package_share_directory("fault_detector_spot"), "maps")

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
        self.bb.register_key("mapping_launch_process", access=py_trees.common.Access.WRITE)
        self.bb.register_key("nav2_launch_process", access=py_trees.common.Access.WRITE)

        if not self.bb.exists("active_map_name"):
            self.bb.active_map_name = None
        if not self.bb.exists("mapping_launch_process"):
            self.bb.mapping_launch_process = None
        if not self.bb.exists("nav2_launch_process"):
            self.bb.nav2_launch_process = None

    def init_ros_publishers(self):
        # ROS publishers
        self.active_map_pub = self.node.create_publisher(String, "active_map", LATCHED_QOS)
        self.map_list_pub = self.node.create_publisher(StringArray, "map_list", LATCHED_QOS)
        self.waypoint_list_pub = self.node.create_publisher(StringArray, "waypoint_list", LATCHED_QOS)

    def _db_path(self, map_name: str):
        return os.path.join(self.recordings_dir, f"{map_name}.db")

    def _publish_active_map(self):
        if self.bb.active_map_name:
            msg = String()
            msg.data = self.bb.active_map_name
            self.active_map_pub.publish(msg)
            self.publish_waypoint_list()

    def update_map_list(self, extra_map: str = None):
        maps = [f[:-3] for f in os.listdir(self.recordings_dir) if f.endswith(".db")]
        if extra_map is not None and extra_map not in maps:
            maps.append(extra_map)

        msg = StringArray()
        msg.names = maps
        self.map_list_pub.publish(msg)

    def start_mapping(self, map_name: str = None, launch_file="slam_merged_launch.py"):
        self.stop_current_process()
        if map_name is None:
            map_name = self.bb.active_map_name
            if map_name is None:
                raise RuntimeError("No active map set to start mapping")
        db_path = self._db_path(map_name)
        args = [
            "ros2", "launch", "fault_detector_spot", launch_file,
            f"database_path:={db_path}", "rviz:=true"
        ]
        proc = subprocess.Popen(args, preexec_fn=os.setsid)
        self.bb.mapping_launch_process = proc
        self.bb.active_map_name = map_name
        self.set_mode_mapping()
        self._publish_active_map()
        return proc

    def start_localization(self, map_name: str = None, launch_file="localization_merged_launch.py"):
        self.stop_current_process()
        if map_name is None:
            map_name = self.bb.active_map_name
            if map_name is None:
                raise RuntimeError("No active map set to start mapping")
        db_path = self._db_path(map_name)
        args = [
            "ros2", "launch", "fault_detector_spot", launch_file,
            f"database_path:={db_path}", "rviz:=true"
        ]
        proc = subprocess.Popen(args, preexec_fn=os.setsid)
        self.bb.mapping_launch_process = proc
        self.bb.active_map_name = map_name
        self.set_mode_localization()
        self._publish_active_map()

        #if not self.nav2_helper.is_running():
            #self.nav2_helper.start()
        return proc

    def stop_current_process(self):
        proc = getattr(self.bb, "mapping_launch_process", None)
        if self.nav2_helper.is_running():
            self.nav2_helper.stop()

        if proc:
            try:
                if self._get_running_mode() == "mapping":
                    self._call_service("pause")
                    self._call_service("publish_map")

                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                proc.wait(timeout=1)


            except Exception as e:
                print(f"Failed to stop RTAB-Map process: {e}")
            finally:
                self.bb.mapping_launch_process = None

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

        # Wait for response
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        if future.done():
            return future.result()
        return False

    def initialize_mapping(self, map_name: str = None, launch_file="slam_merged_launch.py"):
        """
        Creates a new empty RTAB-Map database and corresponding JSON for waypoints,
        then starts SLAM with it.
        """
        # If map_name not given, try to get it from last_command on blackboard
        if map_name is None:
            return

        os.makedirs(self.recordings_dir, exist_ok=True)

        db_path = os.path.join(self.recordings_dir, f"{map_name}.db")
        json_path = os.path.join(self.recordings_dir, f"{map_name}.json")

        # Create empty JSON for waypoints if it does not exist
        if not os.path.exists(json_path):
            import json
            with open(json_path, "w") as f:
                json.dump({"waypoints": []}, f, indent=4)

        # Stop any currently running SLAM/localization
        self.stop_current_process()

        # Build launch command
        args = [
            "ros2", "launch", "fault_detector_spot", launch_file,
            f"database_path:={db_path}", "rviz:=true"
        ]

        # If DB is new or empty, tell rtabmap to delete it on start
        if not os.path.exists(db_path) or os.path.getsize(db_path) == 0:
            args.append("delete_db_on_start:=true")

        # Launch SLAM
        proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        self.bb.mapping_launch_process = proc
        self.bb.active_map_name = map_name

        # Publish active map and map list
        self._publish_active_map()
        self.update_map_list(map_name)

        return proc

    def set_mode_localization(self):
        """Switch RTAB-Map to localization mode via service."""
        self._call_service("/rtabmap/set_mode_localization")

    def set_mode_mapping(self):
        """Switch RTAB-Map to mapping mode via service."""
        self._call_service("/rtabmap/set_mode_mapping")

    def is_rtabmap_running(self) -> bool:
        proc = getattr(self.bb, "mapping_launch_process", None)
        if proc is None:
            return False
        return proc.poll() is None

    def _get_running_mode(self) -> str:
        """
        Returns the current RTAB-Map mode as a string: "mapping" or "localization".
        Defaults to "mapping" if detection fails.
        """
        try:
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
        except Exception:
            return "mapping"

    def change_map(self, map_name: str, slam_launch="slam_merged_launch.py",
                   localization_launch="localization_merged_launch.py"):
        """
        Switch RTAB-Map to a different map while keeping the current mode (mapping or localization).
        If no map_name is given, uses the currently active map.
        """
        if not map_name:
            raise RuntimeError("No map specified and no active map set")

        if not self.is_rtabmap_running():
            self.bb.active_map_name = map_name
            self._publish_active_map()
            return True

        self.stop_current_process()

        # Start process with new map in the current mode
        current_mode = self._get_running_mode()
        if current_mode == "mapping":
            self.start_mapping(map_name, slam_launch)
        elif current_mode == "localization":
            self.start_localization(map_name, localization_launch)
        else:
            raise RuntimeError(f"Unknown current mode '{current_mode}'")

        self.feedback_message = f"Changed to map '{map_name}' in {current_mode} mode"
        return True

    def publish_waypoint_list(self, map_name: str = None):
        if map_name is None:
            map_name = self.bb.active_map_name
        if map_name is None:
            print("No active map set, cannot publish waypoint list")
            return

        json_path = os.path.join(self.recordings_dir, f"{self.bb.active_map_name}.json")
        if not os.path.exists(json_path):
            print(f"No waypoint file found for map '{self.bb.active_map_name}'")
            return

        with open(json_path, "r") as f:
            data = json.load(f)

        waypoints = data.get("waypoints", [])
        waypoint_names = [wp.get("name", "") for wp in waypoints]

        msg = StringArray()
        msg.names = waypoint_names
        self.waypoint_list_pub.publish(msg)

    def add_pose_as_waypoint(self, waypoint_name: str, map_name: str = None, pose: PoseStamped = None):
        """
        Add or update a waypoint in the JSON file for the given map.
        """
        if map_name is None:
            map_name = self.bb.active_map_name
        if map_name is None:
            raise ValueError("No map_name provided and no active map set")
        if waypoint_name is None:
            raise ValueError("waypoint_name must be provided")
        if pose is None:
            raise ValueError("pose must be provided")

        json_path = os.path.join(self.recordings_dir, f"{map_name}.json")

        # Load or create JSON structure
        if os.path.exists(json_path):
            with open(json_path, "r") as f:
                data = json.load(f)
        else:
            data = {"waypoints": []}

        # Pose to dictionary
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

        # Update existing waypoint if found
        updated = False
        for wp in data.get("waypoints", []):
            if wp.get("name") == waypoint_name:
                wp["pose"] = pose_dict
                updated = True
                break

        # Otherwise add new waypoint
        if not updated:
            data["waypoints"].append({
                "name": waypoint_name,
                "pose": pose_dict,
            })

        # Ensure directory exists
        os.makedirs(self.recordings_dir, exist_ok=True)

        # Write back to file
        with open(json_path, "w") as f:
            json.dump(data, f, indent=2)

        # Update waypoint list publisher
        if map_name == self.bb.active_map_name:
            self.publish_waypoint_list(map_name)

        return json_path

    def delete_waypoint(self, waypoint_name: str, map_name: str = None):
        """
        Delete a waypoint by name from the JSON file for the given map.
        """
        if map_name is None:
            map_name = self.bb.active_map_name
        if map_name is None:
            raise ValueError("No map_name provided and no active map set")
        if waypoint_name is None:
            raise ValueError("waypoint_name must be provided")

        json_path = os.path.join(self.recordings_dir, f"{map_name}.json")

        if not os.path.exists(json_path):
            print(f"No waypoint file found for map '{map_name}'")
            return False

        with open(json_path, "r") as f:
            data = json.load(f)

        waypoints = data.get("waypoints", [])
        new_waypoints = [wp for wp in waypoints if wp.get("name") != waypoint_name]

        if len(new_waypoints) == len(waypoints):
            print(f"Waypoint '{waypoint_name}' not found in map '{map_name}'")
            return False

        data["waypoints"] = new_waypoints

        with open(json_path, "w") as f:
            json.dump(data, f, indent=2)

        if map_name == self.bb.active_map_name:
            self.publish_waypoint_list(map_name)

        return True
