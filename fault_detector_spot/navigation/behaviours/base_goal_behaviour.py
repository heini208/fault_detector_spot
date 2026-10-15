"""Behavior-tree adapter for geometric base goal commands."""

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.behaviour_tree.commands.move_to_pose_command import MoveToPoseCommand

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

    RETRY_POSE_COMMAND_ID = CommandID.MOVE_BASE_TO_POSE

    def _start_operation(self):
        command = self._last_command()

        if isinstance(command, MoveToPoseCommand):
            return self.executor.absolute(command)

        if isinstance(command, BaseMoveRelativeCommand):
            return self.executor.relative(command)

        if isinstance(command, BaseToTagCommand):
            return self.executor.tag(command)

        raise RuntimeError(
            "Expected base relative or tag movement command, got "
            f"{type(command).__name__}"
        )


__all__ = ["BaseGoalBehaviour"]
