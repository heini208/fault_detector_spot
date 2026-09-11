"""Behavior-tree adapter for preparing the Spot arm."""

from fault_detector_spot.manipulation.behaviours.move_arm_action import (
    MoveArmAction,
)


class ReadyArmAction(MoveArmAction):
    """Request ArmMovementExecutor.prepare()."""

    def __init__(
        self,
        name: str = "ReadyArmAction",
        robot_name: str = "",
        robot_command_resources=None,
    ):
        super().__init__(
            name,
            robot_name=robot_name,
            robot_command_resources=robot_command_resources,
        )

    def _start_operation(self):
        return self.executor.prepare()


__all__ = ["ReadyArmAction"]
