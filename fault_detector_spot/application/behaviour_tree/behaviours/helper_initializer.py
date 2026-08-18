import py_trees

from fault_detector_spot.mapping.runtime.rtab_helper import RTABHelper
from fault_detector_spot.shared.persistence.runtime_paths import (
    default_map_root,
)
from fault_detector_spot.application.behaviour_tree.behaviours.robot_command_resources import (
    RobotCommandResources,
)


class HelperInitializer(py_trees.behaviour.Behaviour):
    """Initialize resources shared by the behavior-tree runtime."""

    def __init__(self, name: str, node):
        super().__init__(name)
        self.node = node
        self.slam_helper = None
        self.nav2_helper = None
        self.robot_command_resources = RobotCommandResources()

    def setup(self, timeout):
        self.bb_client = self.attach_blackboard_client()

        if not self.node.has_parameter("navigation.map_root"):
            self.node.declare_parameter(
                "navigation.map_root",
                str(default_map_root()),
            )
        configured_map_root = str(
            self.node.get_parameter("navigation.map_root").value
        ).strip()
        map_root = configured_map_root or str(default_map_root())

        self.slam_helper = RTABHelper(
            node=self.node,
            blackboard=self.bb_client,
            maps_dir=map_root,
            launch_file="lidar_rtab_mapping_launch.py",
            nav2_launch_file="nav2_lidar_launch.py",
            nav2_params_file="nav2_lidar_params.yaml",
        )

        self.nav2_helper = self.slam_helper.nav2_helper
        return True

    def initialise(self):
        pass

    def update(self):
        return py_trees.common.Status.SUCCESS

    def close(self):
        """Close shared ROS entities that are not tree children."""
        self.robot_command_resources.close()
