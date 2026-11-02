"""Explicit post-reset stow with current recovery prerequisites."""

from fault_detector_spot.manipulation.arm_movement_result import ArmMovementOutcome, ArmMovementUpdate
from fault_detector_spot.manipulation.behaviours.arm_movement_behaviour import ArmMovementBehaviour


class RecoverArmBehaviour(ArmMovementBehaviour):
    def _start_operation(self):
        allowed, detail = self.robot_command_resources.validate_recovery(self.node)
        if not allowed:
            return ArmMovementUpdate(ArmMovementOutcome.EXECUTION_ERROR, detail)
        return self.executor.stow()
