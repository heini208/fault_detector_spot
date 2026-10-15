"""Absolute odom target and execution settings for an arm or base move."""

from fault_detector_spot.application.behaviour_tree.commands.execution_command import ExecutionCommand
from fault_detector_spot.application.ros.semantic_command_adapter import stamped_pose_to_message


class MoveToPoseCommand(ExecutionCommand):
    def __init__(self, command, stamp):
        super().__init__(command.command_id, stamp)
        self.offset = stamped_pose_to_message(command.offset)
        self.arm_speed_scale = command.arm_speed_scale
        self.walking_profile = command.walking_profile
        self.tag_position_tolerance_m = command.tag_position_tolerance_m
        self.precision_mode = command.precision_mode
        self.precision_orientation_tolerance_rad = command.precision_orientation_tolerance_rad
        self.ignore_environment_collisions = command.ignore_environment_collisions
