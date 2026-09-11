"""Behavior-tree adapter for stowing the Spot arm."""

from fault_detector_spot.manipulation.behaviours.move_arm_action import (
    MoveArmAction,
)


class StowArmAction(MoveArmAction):
    """Request ArmMovementExecutor.stow()."""

    def __init__(
        self,
        name: str = "StowArmAction",
        robot_name: str = "",
        robot_command_resources=None,
    ):
        super().__init__(
            name,
            robot_name=robot_name,
            robot_command_resources=robot_command_resources,
        )

    def _start_operation(self):
        return self.executor.stow()


__all__ = ["StowArmAction"]
