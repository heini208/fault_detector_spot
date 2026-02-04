#!/usr/bin/env python3
import tf_transformations as tf
from fault_detector_spot.behaviour_tree.commands.move_relative_command import MoveRelativeCommand
from geometry_msgs.msg import Quaternion, PoseStamped
from synchros2.tf_listener_wrapper import TFListenerWrapper


class BaseMoveRelativeCommand(MoveRelativeCommand):
    """
    Specialization for Base: Flattens result to SE2 (Yaw only).
    """

    def compute_goal_pose(self, transformer: TFListenerWrapper) -> PoseStamped:
        # Parent does full 3D transform of offset -> target
        generic_pose = super().compute_goal_pose(transformer)

        # Flatten Z
        generic_pose.pose.position.z = 0.0

        # Flatten Orientation to Yaw only
        q_list = [
            generic_pose.pose.orientation.x,
            generic_pose.pose.orientation.y,
            generic_pose.pose.orientation.z,
            generic_pose.pose.orientation.w
        ]
        _, _, yaw = tf.euler_from_quaternion(q_list)
        q_yaw_only = tf.quaternion_from_euler(0.0, 0.0, yaw)

        generic_pose.pose.orientation = Quaternion(
            x=q_yaw_only[0], y=q_yaw_only[1], z=q_yaw_only[2], w=q_yaw_only[3]
        )

        return generic_pose