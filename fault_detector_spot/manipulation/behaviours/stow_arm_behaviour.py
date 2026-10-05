"""Behavior-tree adapter for stowing the Spot arm."""

from py_trees.common import Status

from fault_detector_spot.manipulation.behaviours.arm_movement_behaviour import (
    ArmMovementBehaviour,
)


class StowArmBehaviour(ArmMovementBehaviour):
    """Request ArmMovementExecutor.stow()."""

    def __init__(
        self,
        name: str = "StowArmBehaviour",
        robot_name: str = "",
        robot_command_resources=None,
        preempt: bool = False,
    ):
        super().__init__(
            name,
            robot_name=robot_name,
            robot_command_resources=robot_command_resources,
        )
        self.preempt = preempt

    def update(self):
        if self.preempt and not self._started:
            self._ensure_executor()
            if self.executor.active:
                self.executor.cancel()
                self.feedback_message = "Waiting for confirmed arm stop before stowing"
                return Status.RUNNING
            if self.robot_command_resources.navigation_stopping():
                self.feedback_message = "Waiting for confirmed navigation stop before stowing"
                return Status.RUNNING
        return super().update()

    def _start_operation(self):
        return self.executor.stow()


__all__ = ["StowArmBehaviour"]
