"""Behavior-tree adapter for tag orientation."""

from fault_detector_spot.manipulation.behaviours.arm_movement_behaviour import (
    ArmMovementBehaviour,
)
from fault_detector_spot.manipulation.commands.orient_to_tag_command import (
    OrientToTagCommand,
)


class OrientToTagBehaviour(ArmMovementBehaviour):
    """Orient the active probe to the selected reachable tag."""

    def _start_operation(self):
        command = self._last_command()
        if not isinstance(command, OrientToTagCommand):
            raise RuntimeError(
                "Expected OrientToTagCommand, got "
                f"{type(command).__name__}"
            )
        return self.executor.orient_to_tag(
            command.tag_id,
            command.motion_sensor_id,
        )


__all__ = ["OrientToTagBehaviour"]
