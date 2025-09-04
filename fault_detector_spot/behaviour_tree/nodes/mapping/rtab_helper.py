import os
import signal
import subprocess

from fault_detector_msgs.msg import StringArray
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import py_trees
from fault_detector_spot.behaviour_tree.QOS_PROFILES import LATCHED_QOS


class RTABHelper:
    """
    Helper class for managing RTAB-Map SLAM and Localization processes.
    Integrates with py_trees Blackboard and ROS2 Node for publishing map state.
    """

    def __init__(self, node, blackboard):
        self.node = node
        self.bb = blackboard
        self.recordings_dir = os.path.join(get_package_share_directory("fault_detector_spot"), "maps")

        # Ensure blackboard keys exist
        self.bb.register_key("active_map_name", access=py_trees.common.Access.WRITE)
        self.bb.register_key("mapping_launch_process", access=py_trees.common.Access.WRITE)

        if not self.bb.exists("active_map_name"):
            self.bb.active_map_name = None
        if not self.bb.exists("mapping_launch_process"):
            self.bb.mapping_launch_process = None

        # ROS publishers
        self.active_map_pub = self.node.create_publisher(String, "active_map", LATCHED_QOS)
        self.map_list_pub = self.node.create_publisher(StringArray, "map_list", LATCHED_QOS)

        # Initialize map list from recordings directory
        self.update_map_list()

    def _db_path(self, map_name: str):
        return os.path.join(self.recordings_dir, f"{map_name}.db")

    def _publish_active_map(self):
        if self.bb.active_map_name:
            msg = String()
            msg.data = self.bb.active_map_name
            self.active_map_pub.publish(msg)

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
        return proc

    def stop_current_process(self):
        proc = getattr(self.bb, "mapping_launch_process", None)
        if proc:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except Exception as e:
                print(f"Failed to stop RTAB-Map process: {e}")
            finally:
                self.bb.mapping_launch_process = None

    def restart_mapping(self, map_name: str, launch_file="slam_merged_launch.py"):
        self.start_mapping(map_name, launch_file)

    def restart_localization(self, map_name: str, launch_file="localization_merged_launch.py"):
        self.start_localization(map_name, launch_file)

    def initialize_mapping(self, map_name: str = None, launch_file="slam_merged_launch.py"):
        """
        Creates a new empty RTAB-Map database and corresponding JSON for landmarks,
        then starts SLAM with it.
        """
        # If map_name not given, try to get it from last_command on blackboard
        if map_name is None:
            return

        os.makedirs(self.recordings_dir, exist_ok=True)

        db_path = os.path.join(self.recordings_dir, f"{map_name}.db")
        json_path = os.path.join(self.recordings_dir, f"{map_name}.json")

        # Create empty JSON for landmarks if it does not exist
        if not os.path.exists(json_path):
            import json
            with open(json_path, "w") as f:
                json.dump({"landmarks": []}, f, indent=4)

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
        subprocess.run(
            ["ros2", "param", "set", "/rtabmap", "localization", "true"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )

    def set_mode_mapping(self):
        subprocess.run(
            ["ros2", "param", "set", "/rtabmap", "localization", "false"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )

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
