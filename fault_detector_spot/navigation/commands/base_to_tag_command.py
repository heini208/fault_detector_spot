#!/usr/bin/env python3
from builtin_interfaces.msg import Time
from fault_detector_spot.application.behaviour_tree.commands.move_to_tag_command import (
    MoveToTagCommand,
)
from fault_detector_spot.inspection.geometry.rotation import (
    quaternion_from_euler,
    quaternion_to_rpy,
)
from geometry_msgs.msg import Quaternion, PoseStamped
from synchros2.tf_listener_wrapper import TFListenerWrapper


class BaseToTagCommand(MoveToTagCommand):
    """Tag-relative base movement flattened to SE2."""

    def __init__(
        self,
        command_id: str,
        stamp: Time,
        tag_pose: PoseStamped,
        tag_id: int,
        offset: PoseStamped = None,
        target_frame: str = "odom",
    ):
        super().__init__(
            command_id,
            stamp,
            tag_pose,
            tag_id,
            offset,
            target_frame=target_frame,
        )

    def compute_goal_pose(
        self,
        transformer: TFListenerWrapper,
    ) -> PoseStamped:
        generic_pose = super().compute_goal_pose(transformer)
        generic_pose.pose.position.z = 0.0

        rotated_q = self._rotate_quaternion_into_frame(
            [
                self.offset.pose.orientation.x,
                self.offset.pose.orientation.y,
                self.offset.pose.orientation.z,
                self.offset.pose.orientation.w,
            ],
            self.offset.header.frame_id,
            self.target_frame,
            transformer,
        )
        _, _, yaw = quaternion_to_rpy(
            self._quaternion_data(rotated_q)
        )
        q_yaw_only = quaternion_from_euler("z", yaw)
        generic_pose.pose.orientation = Quaternion(
            x=q_yaw_only.x,
            y=q_yaw_only.y,
            z=q_yaw_only.z,
            w=q_yaw_only.w,
        )
        return generic_pose
