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
        safe_approach: bool = False,
    ):
        super().__init__(
            name,
            robot_name=robot_name,
            robot_command_resources=robot_command_resources,
        )
        self.safe_approach = bool(safe_approach)

    def _start_operation(self):
        speed = (
            self.executor.safe_approach_speed
            if self.safe_approach
            else None
        )
        return self.executor.prepare(speed=speed)


__all__ = ["ReadyArmBehaviour"]
