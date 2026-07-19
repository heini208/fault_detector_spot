"""Resolve a recorded map waypoint into a navigation goal."""

import os

import py_trees
from ament_index_python.packages import get_package_share_directory
from fault_detector_msgs.msg import ComplexCommand
from geometry_msgs.msg import PoseStamped

from fault_detector_spot.inspection.map_repository import MapRepository
from fault_detector_spot.inspection.transform_utils import (
    pose_data_to_pose,
)


class SetWaypointAsGoal(py_trees.behaviour.Behaviour):
    """Set the command goal pose from strict map metadata."""

    def __init__(self, name: str = "SetWaypointAsGoal"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.repository = None
        self.node = None

    def setup(self, **kwargs):
        """Register blackboard access and create the repository."""
        self.node = kwargs.get("node")
        if self.node is None:
            raise RuntimeError(
                "Setup requires a ROS node passed as 'node' kwarg"
            )
        self.blackboard.register_key(
            "last_command",
            access=py_trees.common.Access.WRITE,
        )
        maps_dir = os.path.join(
            get_package_share_directory("fault_detector_spot"),
            "maps",
        )
        self.repository = MapRepository(maps_dir)

    def update(self) -> py_trees.common.Status:
        """Load the requested waypoint and assign its map pose."""
        if (
            not self.blackboard.exists("last_command")
            or self.blackboard.last_command is None
        ):
            self.feedback_message = "No last_command on blackboard"
            return py_trees.common.Status.FAILURE

        command: ComplexCommand = self.blackboard.last_command
        if not command.waypoint_name or not command.map_name:
            self.feedback_message = (
                "No waypoint_name or map_name in last_command"
            )
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
