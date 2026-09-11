"""Behavior-tree adapter for standing the Spot base."""

from fault_detector_spot.navigation.behaviours.base_movement_behaviour import (
    BaseMovementBehaviour,
)


class StandUpBehaviour(BaseMovementBehaviour):
    """Request BaseMovementExecutor.stand()."""

    def __init__(
        self,
        name: str = "StandUpBehaviour",
        robot_name: str = "",
        robot_command_resources=None,
    ):
        super().__init__(
            name,
            robot_name=robot_name,
            robot_command_resources=robot_command_resources,
        )

    def _start_operation(self):
        return self.executor.stand()


__all__ = ["StandUpBehaviour"]
