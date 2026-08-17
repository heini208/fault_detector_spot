#!/usr/bin/env python3
from fault_detector_spot.application.behaviour_tree.commands.move_relative_command import (
    MoveRelativeCommand,
)
from fault_detector_spot.inspection.geometry.rotation import (
    quaternion_from_euler,
    quaternion_to_rpy,
)
from geometry_msgs.msg import Quaternion, PoseStamped
from synchros2.tf_listener_wrapper import TFListenerWrapper


class BaseMoveRelativeCommand(MoveRelativeCommand):
    """Relative base movement flattened to SE2."""

    def compute_goal_pose(
        self,
        transformer: TFListenerWrapper,
    ) -> PoseStamped:
        generic_pose = super().compute_goal_pose(transformer)
        generic_pose.pose.position.z = 0.0

        _, _, yaw = quaternion_to_rpy(
            self._ros_quaternion_data(
                generic_pose.pose.orientation
            )
        )
        q_yaw_only = quaternion_from_euler("z", yaw)
        generic_pose.pose.orientation = Quaternion(
            x=q_yaw_only.x,
            y=q_yaw_only.y,
            z=q_yaw_only.z,
            w=q_yaw_only.w,
        )
        return generic_pose
