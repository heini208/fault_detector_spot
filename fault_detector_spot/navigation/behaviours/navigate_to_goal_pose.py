"""Behavior-tree adapter for prepared waypoint navigation."""

from fault_detector_spot.application.behaviour_tree.behaviours.movement_behaviour import (
    MovementBehaviour,
)
from fault_detector_spot.navigation.waypoint_navigation_executor import (
    WaypointNavigationOutcome,
)


class NavigateToGoalPose(MovementBehaviour):
    """Every invocation stows the arm and prepares height before Nav2 dispatch."""

    RUNNING_OUTCOME = WaypointNavigationOutcome.RUNNING
    SUCCESS_OUTCOME = WaypointNavigationOutcome.SUCCESS

    def __init__(self, name="NavigateToGoalPose", robot_command_resources=None):
        super().__init__(name)
        self.robot_command_resources = robot_command_resources

    def _ensure_executor(self):
        if self.executor is not None:
            return
        resources = self.robot_command_resources
        if resources is None:
            raise RuntimeError("Waypoint navigation requires shared robot command resources")
        self.executor = resources.get_waypoint_navigation_executor(self.node)

    def _start_operation(self):
        return self.executor.navigate(self._last_command().goal_pose)
