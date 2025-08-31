import py_trees
import subprocess

class EnableSLAM(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "EnableSLAM", launch_file: str = "navigation_merged_launch.py"):
        super(EnableSLAM, self).__init__(name)
        self.launch_file = launch_file

    def update(self) -> py_trees.common.Status:
        if self.is_rtabmap_running():
            self.feedback_message = "RTAB-Map node detected, switching to SLAM mode"
            self.switch_to_slam_mode()
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = f"RTAB-Map not running, launching {self.launch_file}"
            if self.start_slam_launch():
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE

    def is_rtabmap_running(self) -> bool:
        """Check if rtabmap node is listed in ros2 node list."""
        try:
            result = subprocess.run(
                ["ros2", "node", "list"],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
            )
            nodes = result.stdout.splitlines()
            return any("rtabmap" in n for n in nodes)
        except Exception:
            return False

    def start_slam_launch(self) -> bool:
        try:
            subprocess.Popen(
                ["ros2", "launch", "fault_detector_spot", self.launch_file],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            return True
        except Exception as e:
            self.feedback_message = f"Failed to launch SLAM: {e}"
            return False

    def switch_to_slam_mode(self):
        try:
            subprocess.run(
                ["ros2", "param", "set", "/rtabmap", "localization", "false"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception as e:
            self.feedback_message = f"Failed to switch to SLAM mode: {e}"
