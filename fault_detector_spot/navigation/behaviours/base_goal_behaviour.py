"""Behavior-tree adapter for geometric base goal commands."""

from copy import deepcopy

from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
from py_trees.common import Status

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
    """Prepare one base goal and dispatch it to BaseMovementExecutor."""

    def _prepare_operation(self):
        command = self._last_command()

        if isinstance(command, BaseToTagCommand):
            if self.tag_state_source is None:
                self.feedback_message = (
                    "Base tag movement requires a tag state source"
                )
                return Status.FAILURE

            tag = self.tag_state_source.visible_snapshot().get(
                int(command.tag_id)
            )
            if tag is None:
                self.feedback_message = (
                    f"Tag {int(command.tag_id)} is not currently visible"
                )
                return Status.FAILURE
            command.tag_pose = deepcopy(tag.pose)

        if not isinstance(
            command,
            (BaseMoveRelativeCommand, BaseToTagCommand),
        ):
            self.feedback_message = (
                "Expected base relative or tag movement command, got "
                f"{type(command).__name__}"
            )
            return Status.FAILURE

        return self._prepare_move_command(
            command,
            final_frame=ODOM_FRAME_NAME,
        )

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
