#!/usr/bin/env python3
import numpy as np

from builtin_interfaces.msg import Time
from fault_detector_spot.behaviour_tree.commands.move_command import MoveCommand
from geometry_msgs.msg import PoseStamped, Quaternion
from synchros2.tf_listener_wrapper import TFListenerWrapper


class MoveRelativeCommand(MoveCommand):
    """
    Represents a movement defined purely by an offset relative to a target frame.
    """

    def __init__(
            self,
            command_id: str,
            stamp: Time,
            offset: PoseStamped = None,
            target_frame: str = "body",
    ):
        super().__init__(command_id, stamp, offset, target_frame)
        self.goal_pose = PoseStamped()  # Cache result

    def compute_goal_pose(self, transformer: TFListenerWrapper) -> PoseStamped:
        """
        Default implementation: Rotates the offset into the target frame and returns it as the goal.
        Subclasses (like Base/Manipulator) can override or extend this logic.
        """
        offset_vec = np.array([
            self.offset.pose.position.x,
            self.offset.pose.position.y,
            self.offset.pose.position.z
        ])

        # 1. Rotate position offset into target frame
        rotated_offset = self._rotate_vector_into_frame(
            offset_vec,
            self.offset.header.frame_id,
            self.target_frame,
            transformer
        )

        # 2. Rotate orientation offset into target frame
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

        # 3. Build Result
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = self.target_frame
        self.goal_pose.header.stamp = self.stamp
        self.goal_pose.pose.position.x = rotated_offset[0]
        self.goal_pose.pose.position.y = rotated_offset[1]
        self.goal_pose.pose.position.z = rotated_offset[2]
        self.goal_pose.pose.orientation = Quaternion(
            x=rotated_q[0], y=rotated_q[1], z=rotated_q[2], w=rotated_q[3]
        )
        return self.goal_pose