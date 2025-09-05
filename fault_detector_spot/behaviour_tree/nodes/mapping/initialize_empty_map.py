import os
import json
import subprocess
import py_trees
from ament_index_python.packages import get_package_share_directory
from fault_detector_msgs.msg import StringArray
from fault_detector_spot.behaviour_tree.QOS_PROFILES import LATCHED_QOS
from fault_detector_spot.behaviour_tree.commands.generic_complex_command import GenericCommand
import signal

from fault_detector_spot.behaviour_tree.nodes.mapping.rtab_helper import RTABHelper
from py_trees.behaviours import Success
from std_msgs.msg import String


class InitializeEmptyMap(py_trees.behaviour.Behaviour):
    """
    Initializes a new RTAB-Map database and corresponding JSON file for waypoints,
    then starts SLAM.
    """

    def __init__(self, name: str = "InitializeEmptyMap", launch_file: str = "slam_merged_launch.py"):
        super().__init__(name)
        self.launch_file = launch_file
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key("last_command", access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        self.node = kwargs.get("node")
        if self.node is None:
            raise RuntimeError("Node must be passed to setup() for ROS publishing")
        self.rtab_helper = RTABHelper(self.node, self.blackboard)

    def update(self) -> py_trees.common.Status:
        if not self.is_command_valid():
            return py_trees.common.Status.FAILURE

        cmd = self.blackboard.last_command
        map_name = cmd.map_name.strip()
        self.rtab_helper.initialize_mapping(map_name, self.launch_file)

        return Success

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
