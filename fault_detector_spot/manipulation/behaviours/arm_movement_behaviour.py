"""Common behavior-tree adapter for arm executor operations."""

from fault_detector_spot.application.behaviour_tree.behaviours.movement_behaviour import (
    MovementBehaviour,
)
from fault_detector_spot.manipulation.arm_movement_executor import (
    ArmMovementOutcome,
)


class ArmMovementBehaviour(MovementBehaviour):
    """Provide one shared ArmMovementExecutor to arm actions."""

    RUNNING_OUTCOME = ArmMovementOutcome.RUNNING
    SUCCESS_OUTCOME = ArmMovementOutcome.SUCCESS

    def __init__(
        self,
        name: str,
        robot_name: str = "",
        tag_state_source=None,
        robot_command_resources=None,
    ):
        super().__init__(name)
        self.robot_name = robot_name
        self.tag_state_source = tag_state_source
        self.robot_command_resources = robot_command_resources

    def _ensure_executor(self) -> None:
        if self.executor is not None:
            return

        if self.robot_command_resources is None:
            raise RuntimeError(
                f"{self.__class__.__name__} requires shared "
                "robot command resources"
            )

        self.executor = (
            self.robot_command_resources.get_arm_movement_executor(
                self.node,
                tag_state_source=self.tag_state_source,
                robot_name=self.robot_name,
            )
        )


__all__ = ["ArmMovementBehaviour"]
