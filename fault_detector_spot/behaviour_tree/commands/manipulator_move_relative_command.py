#!/usr/bin/env python3
from bosdyn.client.frame_helpers import HAND_FRAME_NAME

from builtin_interfaces.msg import Time
from fault_detector_spot.behaviour_tree.commands.move_relative_command import MoveRelativeCommand
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
    ):
        super().__init__(command_id, stamp, offset, target_frame)

    def compute_goal_pose(self, transformer: TFListenerWrapper) -> PoseStamped:
        # Parent computes result = offset rotated into target_frame.
        # For MoveRelative, 'result.pose.orientation' is the offset's orientation in target frame.
        result = super().compute_goal_pose(transformer)
        return result