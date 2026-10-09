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
        self.blackboard.register_key(
            "active_map_name",
            access=py_trees.common.Access.READ,
        )
        for key in ("command_failure_request_id", "command_failure_detail"):
            self.blackboard.register_key(key, access=py_trees.common.Access.WRITE)
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
            return self._fail("No last_command on blackboard")
        command: WaypointCommand = self.blackboard.last_command
        command.goal_pose = None
        if not command.waypoint_name or not command.map_name:
            return self._fail("No waypoint_name or map_name in last_command", command)
        active_map = (
            self.blackboard.active_map_name
            if self.blackboard.exists("active_map_name") else None
        )
        if not active_map:
            return self._fail("Cannot move to waypoint: no active map", command)
        if command.map_name != active_map:
            return self._fail(
                f"Waypoint '{command.waypoint_name}' belongs to map "
                f"'{command.map_name}', but the active map is '{active_map}'",
                command,
            )
        try:
            waypoint = self.repository.get_waypoint(
                command.map_name,
                command.waypoint_name,
            )
        except (FileNotFoundError, OSError, ValueError) as exception:
            return self._fail(str(exception), command)
        if waypoint is None:
            return self._fail(
                f"Waypoint '{command.waypoint_name}' not found "
                f"in map '{command.map_name}'",
                command,
            )
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose = pose_data_to_pose(waypoint.pose_map)
        command.goal_pose = goal
        self.feedback_message = (
            f"Set goal_pose to waypoint '{command.waypoint_name}'"
        )
        return py_trees.common.Status.SUCCESS

    def _fail(self, detail, command=None):
        self.feedback_message = detail
        self.blackboard.command_failure_request_id = (
            command.request_id if command is not None else ""
        )
        self.blackboard.command_failure_detail = detail
        return py_trees.common.Status.FAILURE
