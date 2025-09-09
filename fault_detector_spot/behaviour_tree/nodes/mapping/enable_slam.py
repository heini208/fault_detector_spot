import os
import py_trees
import subprocess

from ament_index_python import get_package_share_directory
from fault_detector_spot.behaviour_tree.nodes.mapping.rtab_helper import RTABHelper
from fault_detector_spot.behaviour_tree.nodes.mapping.slam_toolbox_helper import SlamToolboxHelper
from py_trees.blackboard import Blackboard


class EnableSLAM(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "EnableSLAM", launch_file: str = "slam_merged_launch.py"):
        super().__init__(name)
        self.launch_file = launch_file
        self.blackboard = self.attach_blackboard_client(name=name)

    def setup(self, **kwargs):
        self.slam_helper = SlamToolboxHelper(kwargs.get("node"), self.blackboard, launch_file=self.launch_file)
        pass

    def update(self) -> py_trees.common.Status:
        if not self.blackboard.active_map_name:
            self.feedback_message = "No active map set, cannot enable SLAM"
            return py_trees.common.Status.FAILURE

        self.feedback_message = f"Launching {self.launch_file} for SLAM"
        self.slam_helper.start_mapping_from_existing()
        if self.slam_helper.is_slam_running():
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
