"""Behavior-tree adapter for geometric arm goal commands."""

from fault_detector_spot.manipulation.behaviours.arm_movement_behaviour import (
    ArmMovementBehaviour,
)
from fault_detector_spot.manipulation.commands.manipulator_move_relative_command import (
    ManipulatorMoveRelativeCommand,
)
from fault_detector_spot.manipulation.commands.manipulator_to_tag_command import (
    ManipulatorToTagCommand,
)


class ArmGoalBehaviour(ArmMovementBehaviour):
    """Dispatch one arm goal to ArmMovementExecutor."""

    def _start_operation(self):
        command = self._last_command()

        if isinstance(command, ManipulatorMoveRelativeCommand):
            return self.executor.relative(command)

        if isinstance(command, ManipulatorToTagCommand):
            return self.executor.tag_probe(command)

        raise RuntimeError(
            "Expected manipulator relative or tag movement command, got "
            f"{type(command).__name__}"
        )


__all__ = ["ArmGoalBehaviour"]
