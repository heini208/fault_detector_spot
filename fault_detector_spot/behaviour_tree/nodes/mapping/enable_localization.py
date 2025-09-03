import py_trees
import subprocess
import os
from py_trees.blackboard import Blackboard


class EnableLocalization(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "EnableLocalization", launch_file: str = "localization_merged_launch.py"):
        super().__init__(name)
        self.launch_file = launch_file

        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key("active_map_name", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("mapping_launch_process", access=py_trees.common.Access.WRITE)

        if "active_map_name" not in Blackboard.storage:
            self.blackboard.active_map_name = None
        if "mapping_launch_process" not in Blackboard.storage:
            self.blackboard.mapping_launch_process = None

    def update(self) -> py_trees.common.Status:
        if self.is_rtabmap_running():
            self.feedback_message = "RTAB-Map detected, switching to Localization mode"
            self.switch_to_localization_mode()
            if self.blackboard.active_map_name:
                self.set_database_path(self.blackboard.active_map_name)
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = f"Launching {self.launch_file} for Localization"
            proc = self.start_localization_launch()
            if proc:
                self.blackboard.mapping_launch_process = proc
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE

    def is_rtabmap_running(self) -> bool:
        try:
            result = subprocess.run(
                ["ros2", "node", "list"],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
            )
            return any("rtabmap" in n for n in result.stdout.splitlines())
        except Exception:
            return False

    def start_localization_launch(self):
        try:
            return subprocess.Popen(
                ["ros2", "launch", "fault_detector_spot", self.launch_file],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid
            )
        except Exception as e:
            self.feedback_message = f"Failed to launch Localization: {e}"
            return None

    def switch_to_localization_mode(self):
        subprocess.run(
            ["ros2", "param", "set", "/rtabmap", "localization", "true"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )

    def set_database_path(self, map_name: str):
        subprocess.run(
            ["ros2", "param", "set", "/rtabmap", "database_path", map_name],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
