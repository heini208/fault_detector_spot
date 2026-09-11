"""Common behavior-tree adapter for base executor operations."""

from fault_detector_spot.application.behaviour_tree.behaviours.movement_behaviour import (
    MovementBehaviour,
)
from fault_detector_spot.navigation.base_movement_executor import (
    BaseMovementOutcome,
)


class BaseMovementBehaviour(MovementBehaviour):
    """Provide one shared BaseMovementExecutor to base behaviors."""

    RUNNING_OUTCOME = BaseMovementOutcome.RUNNING
    SUCCESS_OUTCOME = BaseMovementOutcome.SUCCESS

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
            self.robot_command_resources.get_base_movement_executor(
                self.node,
                tag_state_source=self.tag_state_source,
                robot_name=self.robot_name,
            )
        )


__all__ = ["BaseMovementBehaviour"]
