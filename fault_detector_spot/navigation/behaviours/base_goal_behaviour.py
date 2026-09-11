"""Behavior-tree adapter for geometric base goal commands."""

from fault_detector_spot.navigation.behaviours.base_movement_behaviour import (
    BaseMovementBehaviour,
)
from fault_detector_spot.navigation.commands.base_move_relative_command import (
    BaseMoveRelativeCommand,
)
from fault_detector_spot.navigation.commands.base_to_tag_command import (
    BaseToTagCommand,
)


class BaseGoalBehaviour(BaseMovementBehaviour):
    """Dispatch one base goal to BaseMovementExecutor."""

    def _start_operation(self):
        command = self._last_command()

        if isinstance(command, BaseMoveRelativeCommand):
            return self.executor.relative(command)

        if isinstance(command, BaseToTagCommand):
            return self.executor.tag(command)

        raise RuntimeError(
            "Expected base relative or tag movement command, got "
            f"{type(command).__name__}"
        )


__all__ = ["BaseGoalBehaviour"]
