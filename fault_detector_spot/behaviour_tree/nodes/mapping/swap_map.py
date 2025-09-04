import os
import signal
import subprocess
import py_trees
from fault_detector_spot.behaviour_tree.QOS_PROFILES import LATCHED_QOS
from fault_detector_spot.behaviour_tree.nodes.mapping.rtab_helper import RTABHelper
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

        self.blackboard.register_key("last_command", access=py_trees.common.Access.READ)
        self.rtab_helper = RTABHelper(self.node, self.blackboard)
        return True

    def _validate_last_command(self):
        last_command = getattr(self.blackboard, "last_command", None)
        if not last_command or not getattr(last_command, "map_name", None):
            self.feedback_message = "No map_name in last_command"
            return None
        return last_command.map_name

    def update(self):
        requested_map = self._validate_last_command()
        if not requested_map:
            return py_trees.common.Status.FAILURE

        current_map = getattr(self.blackboard, "active_map_name", None)
        if requested_map == current_map:
            self.feedback_message = f"Map '{requested_map}' already active"
            return py_trees.common.Status.SUCCESS

        self.rtab_helper.change_map(requested_map, self.slam_launch, self.localization_launch)

