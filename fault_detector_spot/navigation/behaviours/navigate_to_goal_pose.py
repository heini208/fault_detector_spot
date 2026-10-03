"""Behavior-tree adapter for prepared waypoint navigation."""

from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

from fault_detector_spot.application.behaviour_tree.behaviours.movement_behaviour import (
    MovementBehaviour,
)
from fault_detector_spot.navigation.waypoint_navigation_executor import (
    WaypointNavigationExecutor, WaypointNavigationOutcome,
)


class NavigateToGoalPose(MovementBehaviour):
    """Every invocation stows the arm and prepares height before Nav2 dispatch."""

    RUNNING_OUTCOME = WaypointNavigationOutcome.RUNNING
    SUCCESS_OUTCOME = WaypointNavigationOutcome.SUCCESS

    def __init__(self, name="NavigateToGoalPose", robot_command_resources=None):
        super().__init__(name)
        self.robot_command_resources = robot_command_resources
        self._action_client = None

    def _ensure_executor(self):
        if self.executor is not None:
            return
        resources = self.robot_command_resources
        if resources is None:
            raise RuntimeError("Waypoint navigation requires shared robot command resources")
        self._action_client = ActionClient(self.node, NavigateToPose, "/navigate_to_pose")
        self.executor = WaypointNavigationExecutor(
            resources.get_arm_movement_executor(self.node),
            resources.get_base_movement_executor(self.node),
            self._action_client,
            stamp_now=lambda: self.node.get_clock().now().to_msg(),
        )

    def _start_operation(self):
        return self.executor.navigate(self._last_command().goal_pose)

    def shutdown(self):
        super().shutdown()
        if self._action_client is not None:
            self._action_client.destroy()
            self._action_client = None
