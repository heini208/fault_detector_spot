#!/usr/bin/env python3
import tf_transformations as tf
from fault_detector_spot.behaviour_tree.commands.move_to_tag_command import MoveToTagCommand
from geometry_msgs.msg import Quaternion, PoseStamped
from synchros2.tf_listener_wrapper import TFListenerWrapper


class BaseToTagCommand(MoveToTagCommand):
    """
    Specialization for Base: Flattens result to SE2 (Yaw only).
    """

    def compute_goal_pose(self, transformer: TFListenerWrapper) -> PoseStamped:
        # Get generic 3D result from parent
        generic_pose = super().compute_goal_pose(transformer)

        # Apply specific Base logic:
        generic_pose.pose.position.z = 0.0

        # 2. Compute Yaw-only orientation from Offset

        q_offset = [
            self.offset.pose.orientation.x,
            self.offset.pose.orientation.y,
            self.offset.pose.orientation.z,
            self.offset.pose.orientation.w
        ]

        rotated_q = self._rotate_quaternion_into_frame(
            q_offset,
            self.offset.header.frame_id,
            self.target_frame,
            transformer
        )

        _, _, yaw = tf.euler_from_quaternion(rotated_q)
        q_yaw_only = tf.quaternion_from_euler(0.0, 0.0, yaw)

        generic_pose.pose.orientation = Quaternion(
            x=q_yaw_only[0], y=q_yaw_only[1], z=q_yaw_only[2], w=q_yaw_only[3]
        )

        return generic_pose