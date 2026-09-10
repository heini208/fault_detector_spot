import py_trees

from fault_detector_spot.mapping.runtime.rtab_helper import RTABHelper
from fault_detector_spot.shared.persistence.runtime_paths import (
    default_map_root,
)
from fault_detector_spot.application.behaviour_tree.behaviours.robot_command_resources import (
    RobotCommandResources,
)
from fault_detector_spot.sensing.tag_state_source import TagStateSource


class HelperInitializer(py_trees.behaviour.Behaviour):
    """Initialize resources shared by the behavior-tree runtime."""

    def __init__(self, name: str, node):
        super().__init__(name)
        self.node = node
        self.slam_helper = None
        self.nav2_helper = None
        self.tag_state_source = None
        self.robot_command_resources = RobotCommandResources()

    def setup(self, timeout):
        self.bb_client = self.attach_blackboard_client()
        self.robot_command_resources.get_arm_state_source(self.node)

        if not self.node.has_parameter(
            "tag_sensing.state_timeout_sec"
        ):
            self.node.declare_parameter(
                "tag_sensing.state_timeout_sec",
                1.5,
            )
        tag_state_timeout_sec = float(
            self.node.get_parameter(
                "tag_sensing.state_timeout_sec"
            ).value
        )
        if self.tag_state_source is None:
            self.tag_state_source = TagStateSource(
                self.node,
                stale_after_sec=tag_state_timeout_sec,
            )

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
        try:
            if self.slam_helper is not None:
                self.slam_helper.close()
        finally:
            try:
                if self.tag_state_source is not None:
                    self.tag_state_source.destroy()
                    self.tag_state_source = None
            finally:
                self.robot_command_resources.close()
