"""Execute a stationary height change using the shared base executor."""

from fault_detector_spot.navigation.behaviours.base_movement_behaviour import (
    BaseMovementBehaviour,
)


class ChangeBodyHeightBehaviour(BaseMovementBehaviour):
    def _start_operation(self):
        return self.executor.change_height(self._last_command().body_height_m)
