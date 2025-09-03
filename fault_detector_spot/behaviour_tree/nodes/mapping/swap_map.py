import os
import signal
import subprocess
import py_trees
from fault_detector_spot.behaviour_tree.QOS_PROFILES import LATCHED_QOS
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


class SwapMap(py_trees.behaviour.Behaviour):
    """
    Swap/load a map in RTAB-Map by restarting SLAM or localization with the new database.
    If RTAB-Map is not running, just set the active map and publish it.
    Uses different launch files depending on actual running mode.
    Updates blackboard variable `active_map_name`.
    """

    def __init__(self, name="SwapMap",
                 slam_launch="slam_merged_launch.py",
                 localization_launch="localization_launch.py"):
        super().__init__(name)
        self.slam_launch = slam_launch
        self.localization_launch = localization_launch
        self.blackboard = self.attach_blackboard_client()
        self.recordings_dir = os.path.join(
            get_package_share_directory("fault_detector_spot"), "maps"
        )

    def setup(self, **kwargs):
        node = kwargs.get("node")
        if node is None:
            raise RuntimeError("Node must be passed to setup() for ROS publishing")
        self.node = node
        self.map_pub = self.node.create_publisher(String, "active_map", LATCHED_QOS)

        self.blackboard.register_key("last_command", access=py_trees.common.Access.READ)
        self.blackboard.register_key("active_map_name", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("mapping_launch_process", access=py_trees.common.Access.WRITE)

        if self.blackboard.active_map_name is None:
            self.blackboard.active_map_name = None
        return True

    def _validate_last_command(self):
        last_command = getattr(self.blackboard, "last_command", None)
        if not last_command or not getattr(last_command, "map_name", None):
            self.feedback_message = "No map_name in last_command"
            return None
        return last_command.map_name

    def _stop_process(self):
        """Stop the running SLAM/localization process if any."""
        proc = getattr(self.blackboard, "mapping_launch_process", None)
        if proc:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except Exception as e:
                self.feedback_message = f"Failed to stop RTAB-Map: {e}"
            finally:
                self.blackboard.mapping_launch_process = None

    def _is_rtabmap_running(self) -> bool:
        """Check if RTAB-Map node exists."""
        try:
            result = subprocess.run(
                ["ros2", "node", "list"],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
            )
            return any("rtabmap" in n for n in result.stdout.splitlines())
        except Exception:
            return False

    def _get_running_mode(self) -> str:
        """Check RTAB-Map node to see if it's in localization or mapping mode."""
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

    def _start_process(self, map_name: str, mode: str):
        db_path = os.path.join(self.recordings_dir, f"{map_name}.db")

        # Choose launch file based on captured mode
        launch_file = self.slam_launch if mode == "mapping" else self.localization_launch

        args = [
            "ros2", "launch", "fault_detector_spot", launch_file,
            f"database_path:={db_path}", "rviz:=true"
        ]

        try:
            proc = subprocess.Popen(
                args,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
                env=os.environ.copy()
            )
            self.blackboard.mapping_launch_process = proc
            self.blackboard.active_map_name = map_name
            self._publish_active_map(map_name)

            self.feedback_message = f"Started {mode} with map '{map_name}'"
            return True
        except Exception as e:
            self.feedback_message = f"Failed to start RTAB-Map: {e}"
            return False

    def _publish_active_map(self, map_name: str):
        msg = String()
        msg.data = map_name
        self.map_pub.publish(msg)

    def update(self):
        requested_map = self._validate_last_command()
        if not requested_map:
            return py_trees.common.Status.FAILURE

        current_map = getattr(self.blackboard, "active_map_name", None)
        if requested_map == current_map:
            self.feedback_message = f"Map '{requested_map}' already active"
            return py_trees.common.Status.SUCCESS

        if self._is_rtabmap_running():
            # Capture mode before stopping
            mode = self._get_running_mode()

            # Stop current process
            self._stop_process()

            # Start process with the new map using captured mode
            success = self._start_process(requested_map, mode)
            return py_trees.common.Status.SUCCESS if success else py_trees.common.Status.FAILURE
        else:
            # RTAB-Map not running — just update blackboard and publish
            self.blackboard.active_map_name = requested_map
            self._publish_active_map(requested_map)
            self.feedback_message = f"RTAB-Map not running, set active map to '{requested_map}'"
            return py_trees.common.Status.SUCCESS

