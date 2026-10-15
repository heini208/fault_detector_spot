"""Behavior-tree adapter for geometric arm goal commands."""

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.behaviour_tree.commands.move_to_pose_command import MoveToPoseCommand

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

    def __init__(
        self,
        name: str,
        robot_name: str = "",
        tag_state_source=None,
        robot_command_resources=None,
        safe_approach: bool = False,
    ):
        super().__init__(
            name,
            robot_name=robot_name,
            tag_state_source=tag_state_source,
            robot_command_resources=robot_command_resources,
        )
        self.safe_approach = bool(safe_approach)

    RETRY_POSE_COMMAND_ID = CommandID.MOVE_ARM_TO_POSE

    def _start_operation(self):
        command = self._last_command()
        speed = (
            self.executor.safe_approach_speed
            if self.safe_approach
            else None
        )

        if isinstance(command, MoveToPoseCommand):
            return self.executor.absolute(command)

        if isinstance(command, ManipulatorMoveRelativeCommand):
            return self.executor.relative(command, speed=speed)

        if isinstance(command, ManipulatorToTagCommand):
            return self.executor.tag_probe(command, speed=speed)

        raise RuntimeError(
            "Expected manipulator relative or tag movement command, got "
            f"{type(command).__name__}"
        )


__all__ = ["ArmGoalBehaviour"]
