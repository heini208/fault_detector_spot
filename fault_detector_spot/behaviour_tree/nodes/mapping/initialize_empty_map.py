import os
import json
import subprocess
import py_trees
from ament_index_python.packages import get_package_share_directory
from fault_detector_msgs.msg import StringArray
from fault_detector_spot.behaviour_tree.QOS_PROFILES import LATCHED_QOS
from fault_detector_spot.behaviour_tree.commands.generic_complex_command import GenericCommand
import signal

from std_msgs.msg import String


class InitializeEmptyMap(py_trees.behaviour.Behaviour):
    """
    Initializes a new RTAB-Map database and corresponding JSON file for landmarks,
    then starts SLAM.
    """

    def __init__(self, name: str = "InitializeEmptyMap", launch_file: str = "slam_merged_launch.py"):
        super().__init__(name)
        self.launch_file = launch_file
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key("last_command", access=py_trees.common.Access.READ)
        self.blackboard.register_key("active_map_name", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("mapping_launch_process", access=py_trees.common.Access.WRITE)

        self.recordings_dir = os.path.join(
            get_package_share_directory("fault_detector_spot"), "maps"
        )
        self.map_list_publisher = None
        self.active_map_publisher = None

    def setup(self, **kwargs):
        self.node = kwargs.get("node")
        if self.node is None:
            raise RuntimeError("Node must be passed to setup() for ROS publishing")
        self.map_list_publisher = self.node.create_publisher(StringArray, "map_list", LATCHED_QOS)
        self.active_map_publisher = self.node.create_publisher(String, 'active_map', LATCHED_QOS)

    def update(self) -> py_trees.common.Status:
        if not self.is_command_valid():
            return py_trees.common.Status.FAILURE

        cmd = self.blackboard.last_command
        map_name = cmd.map_name.strip()
        db_path, json_path = self.create_map_files(map_name)
        self.publish_map_list(extra_map=map_name)

        # If SLAM is already running on a different map, stop it
        if self.blackboard.mapping_launch_process:
            self._stop_launch()

        # Launch SLAM
        proc = self._start_slam(db_path)
        if proc:
            self.blackboard.mapping_launch_process = proc
            self.blackboard.active_map_name = map_name
            self._publish_active_map(map_name)
            self.feedback_message = f"Started SLAM with map {map_name}"
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def create_map_files(self, map_name: str):
        os.makedirs(self.recordings_dir, exist_ok=True)
        db_path = os.path.join(self.recordings_dir, f"{map_name}.db")
        json_path = os.path.join(self.recordings_dir, f"{map_name}.json")

        # JSON for landmarks
        if not os.path.exists(json_path):
            with open(json_path, "w") as f:
                json.dump({"landmarks": []}, f, indent=4)

        return db_path, json_path

    def _start_slam(self, db_path: str):
        # Start command
        args = [
            "ros2", "launch", "fault_detector_spot", self.launch_file,
            f"database_path:={db_path}", "rviz:=true"
        ]

        # Only pass delete_db_on_start if DB is new
        if not os.path.exists(db_path) or os.path.getsize(db_path) == 0:
            args.append(f"delete_db_on_start:=true")

        try:
            return subprocess.Popen(
                args,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
                env=os.environ.copy()
            )
        except Exception as e:
            self.feedback_message = f"Failed to start SLAM: {e}"
            return None

    def _stop_launch(self):
        if self.blackboard.mapping_launch_process:
            try:
                os.killpg(os.getpgid(self.blackboard.mapping_launch_process.pid), signal.SIGINT)
                self.blackboard.mapping_launch_process = None
            except Exception as e:
                self.feedback_message = f"Failed to stop SLAM launch: {e}"

    def is_command_valid(self) -> bool:
        if not self.blackboard.exists("last_command") or self.blackboard.last_command is None:
            self.feedback_message = "No last_command on blackboard"
            return False

        cmd = self.blackboard.last_command
        if not isinstance(cmd, GenericCommand):
            self.feedback_message = f"Expected GenericCommand, got {type(cmd).__name__}"
            return False

        if not cmd.map_name or cmd.map_name.strip() == "":
            self.feedback_message = "No map_name provided in last_command"
            return False

        return True

    def publish_map_list(self, extra_map: str = None):
        if self.map_list_publisher is None:
            return

        map_files = [
            f[:-3] for f in sorted(os.listdir(self.recordings_dir)) if f.endswith(".db")
        ]

        if extra_map is not None and extra_map not in map_files:
            map_files.append(extra_map)

        msg = StringArray()
        msg.names = map_files
        self.map_list_publisher.publish(msg)

    def _publish_active_map(self, map_name: str):
        msg = String()
        msg.data = map_name
        self.active_map_publisher.publish(msg)
