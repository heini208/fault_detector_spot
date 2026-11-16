#!/usr/bin/env python3
from math import radians
from bosdyn.client.frame_helpers import HAND_FRAME_NAME

from builtin_interfaces.msg import Time
from fault_detector_spot.application.behaviour_tree.commands.move_relative_command import (
    MoveRelativeCommand,
)
from fault_detector_spot.shared.geometry.rotation import (
    quaternion_from_rotation,
    rotation_from_quaternion,
)
from geometry_msgs.msg import PoseStamped
from synchros2.tf_listener_wrapper import TFListenerWrapper


class ManipulatorMoveRelativeCommand(MoveRelativeCommand):
    """
    Manipulator command to move relative to a specific frame (default: hand frame),
    """

    def __init__(
            self,
            command_id: str,
            stamp: Time,
            offset: PoseStamped = None,
            target_frame: str = HAND_FRAME_NAME,  # Default relative to hand
            arm_speed_scale: float = 1.0,
            tag_position_tolerance_m: float = 0.01,
            precision_mode: bool = False,
            precision_orientation_tolerance_rad: float = radians(2.0),
    ):
        super().__init__(command_id, stamp, offset, target_frame)
        self.arm_speed_scale = arm_speed_scale
        self.tag_position_tolerance_m = tag_position_tolerance_m
        self.precision_mode = precision_mode
        self.precision_orientation_tolerance_rad = precision_orientation_tolerance_rad

    def _rotate_quaternion_into_frame(
        self, quat_xyzw, source_frame, target_frame, transformer: TFListenerWrapper,
    ):
        """Change the axes of a rotation delta without adding frame orientation."""
        delta = rotation_from_quaternion(self._quaternion_data(quat_xyzw))
        if source_frame != target_frame:
            basis = self._frame_rotation(source_frame, target_frame, transformer)
            delta = basis * delta * basis.inv()
        result = quaternion_from_rotation(delta)
        return [result.x, result.y, result.z, result.w]
