"""Behavior-tree adapter for live surface orientation."""

from fault_detector_spot.manipulation.behaviours.arm_movement_behaviour import (
    ArmMovementBehaviour,
)
from fault_detector_spot.manipulation.commands.orient_to_surface_command import (
    OrientToSurfaceCommand,
)


class OrientToSurfaceBehaviour(ArmMovementBehaviour):
    """Orient the active probe to the front-facing live surface."""

    def _start_operation(self):
        command = self._last_command()
        if not isinstance(command, OrientToSurfaceCommand):
            raise RuntimeError(
                "Expected OrientToSurfaceCommand, got "
                f"{type(command).__name__}"
            )
        return self.executor.orient_to_surface(command.motion_sensor_id)


__all__ = ["OrientToSurfaceBehaviour"]
