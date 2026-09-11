"""Behavior-tree adapter for preparing the Spot arm."""

from fault_detector_spot.manipulation.behaviours.arm_movement_behaviour import (
    ArmMovementBehaviour,
)


class ReadyArmBehaviour(ArmMovementBehaviour):
    """Request ArmMovementExecutor.prepare()."""

    def __init__(
        self,
        name: str = "ReadyArmBehaviour",
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


__all__ = ["ReadyArmBehaviour"]
