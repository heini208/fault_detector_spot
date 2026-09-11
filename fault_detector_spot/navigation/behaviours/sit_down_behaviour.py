"""Behavior-tree adapter for sitting the Spot base."""

from fault_detector_spot.navigation.behaviours.base_movement_behaviour import (
    BaseMovementBehaviour,
)


class SitDownBehaviour(BaseMovementBehaviour):
    """Request BaseMovementExecutor.sit()."""

    def __init__(
        self,
        name: str = "SitDownBehaviour",
        robot_name: str = "",
        robot_command_resources=None,
    ):
        super().__init__(
            name,
            robot_name=robot_name,
            robot_command_resources=robot_command_resources,
        )

    def _start_operation(self):
        return self.executor.sit()


__all__ = ["SitDownBehaviour"]
