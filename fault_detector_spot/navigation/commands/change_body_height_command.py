"""Stationary height request passed through the behavior tree."""

from fault_detector_spot.application.behaviour_tree.commands.execution_command import (
    ExecutionCommand,
)
from fault_detector_spot.navigation.body_height import validate_body_height


class ChangeBodyHeightCommand(ExecutionCommand):
    def __init__(self, command_id, stamp, body_height_m):
        super().__init__(command_id, stamp)
        self.body_height_m = validate_body_height(body_height_m)
