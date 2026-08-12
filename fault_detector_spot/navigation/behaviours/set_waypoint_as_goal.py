"""Resolve a recorded map waypoint into a navigation goal."""

import py_trees
from geometry_msgs.msg import PoseStamped

from fault_detector_spot.navigation.commands.waypoint_command import WaypointCommand
from fault_detector_spot.mapping.repository.map_repository import MapRepository
from fault_detector_spot.shared.geometry.transforms import pose_data_to_pose
from fault_detector_spot.shared.persistence.runtime_paths import default_map_root


class SetWaypointAsGoal(py_trees.behaviour.Behaviour):
    """Set the command goal pose from strict map metadata."""

    def __init__(self, name: str = "SetWaypointAsGoal"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.repository = None
        self.node = None

    def setup(self, **kwargs):
        self.node = kwargs.get("node")
        if self.node is None:
            raise RuntimeError("Setup requires a ROS node passed as 'node' kwarg")
        self.blackboard.register_key(
            "last_command",
            access=py_trees.common.Access.WRITE,
        )
        if not self.node.has_parameter("navigation.map_root"):
            self.node.declare_parameter(
                "navigation.map_root",
                str(default_map_root()),
            )
        configured_root = str(
            self.node.get_parameter("navigation.map_root").value
        ).strip()
        self.repository = MapRepository(
            configured_root or str(default_map_root())
        )

    def update(self) -> py_trees.common.Status:
        if (
            not self.blackboard.exists("last_command")
            or self.blackboard.last_command is None
        ):
            self.feedback_message = "No last_command on blackboard"
            return py_trees.common.Status.FAILURE
        command: WaypointCommand = self.blackboard.last_command
        if not command.waypoint_name or not command.map_name:
            self.feedback_message = "No waypoint_name or map_name in last_command"
            return py_trees.common.Status.FAILURE
        try:
            waypoint = self.repository.get_waypoint(
                command.map_name,
                command.waypoint_name,
            )
        except (FileNotFoundError, OSError, ValueError) as exception:
            self.feedback_message = str(exception)
            return py_trees.common.Status.FAILURE
        if waypoint is None:
            self.feedback_message = (
                f"Waypoint '{command.waypoint_name}' not found "
                f"in map '{command.map_name}'"
            )
            return py_trees.common.Status.FAILURE
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose = pose_data_to_pose(waypoint.pose_map)
        command.goal_pose = goal
        self.feedback_message = (
            f"Set goal_pose to waypoint '{command.waypoint_name}'"
        )
        return py_trees.common.Status.SUCCESS
