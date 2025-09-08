import py_trees
from fault_detector_spot.behaviour_tree.commands.generic_complex_command import GenericCommand

from fault_detector_spot.behaviour_tree.nodes.mapping.slam_toolbox_helper import SlamToolboxHelper
from py_trees.behaviours import Success


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

        self.slam_helper = SlamToolboxHelper(self.node, self.blackboard)

    def update(self) -> py_trees.common.Status:
        if not self.is_command_valid():
            return py_trees.common.Status.FAILURE

        cmd = self.blackboard.last_command
        map_name = cmd.map_name.strip()
        self.slam_helper.start_mapping(map_name, self.launch_file)

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
