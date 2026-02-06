#!/usr/bin/env python3
import numpy as np

import tf2_geometry_msgs
from builtin_interfaces.msg import Time
from fault_detector_spot.behaviour_tree.commands.move_command import MoveCommand
from geometry_msgs.msg import PoseStamped
from synchros2.tf_listener_wrapper import TFListenerWrapper


class MoveToTagCommand(MoveCommand):
    """
    Represents a movement to a specific tag (or initial goal pose) with an added offset.
    """

    def __init__(
            self,
            command_id: str,
            stamp: Time,
            tag_pose: PoseStamped,
            tag_id: int,
            offset: PoseStamped = None,
            target_frame: str = "body",
    ):
        super().__init__(command_id, stamp, offset, target_frame)
        self.tag_pose = tag_pose
        self.tag_id = tag_id

    def transform_tag_to_target_frame(self, transformer: TFListenerWrapper) -> PoseStamped:
        """
        Transforms the initial goal pose (tag) to the target frame.
        """
        if transformer:
            tf_to_target = transformer.lookup_a_tform_b(
                self.target_frame, self.tag_pose.header.frame_id, timeout_sec=2
            )
            tag_in_target = tf2_geometry_msgs.do_transform_pose_stamped(
                transform=tf_to_target, pose=self.tag_pose
            )
            return tag_in_target
        else:
            # Assuming already in target frame if no transformer (risky but fallback)
            return self.tag_pose

    def add_offset_to_tag_pose(self, tag_in_target: PoseStamped, transformer: TFListenerWrapper) -> PoseStamped:
        """
        Adds the rotated offset to the transformed tag pose.
        """
        # Rotate offset into target frame
        offset_vec = np.array([
            self.offset.pose.position.x,
            self.offset.pose.position.y,
            self.offset.pose.position.z
        ])
        rotated_offset = self._rotate_vector_into_frame(
            offset_vec,
            self.offset.header.frame_id,
            self.target_frame,
            transformer
        )

        # Add positions
        final_x = tag_in_target.pose.position.x + rotated_offset[0]
        final_y = tag_in_target.pose.position.y + rotated_offset[1]
        final_z = tag_in_target.pose.position.z + rotated_offset[2]

        result = PoseStamped()
        result.header.frame_id = self.target_frame
        result.header.stamp = self.stamp
        result.pose.position.x = final_x
        result.pose.position.y = final_y
        result.pose.position.z = final_z
        result.pose.orientation = tag_in_target.pose.orientation
        return result

    def compute_goal_pose(self, transformer: TFListenerWrapper) -> PoseStamped:
        """
        Generic Logic:
        1. Transform initial_goal_pose (tag) to target_frame.
        2. Rotate offset to target_frame.
        3. Add offset to transformed tag pose.
        """
        # 1. Transform tag to target frame
        tag_in_target = self.transform_tag_to_target_frame(transformer=transformer)
        result = self.add_offset_to_tag_pose(tag_in_target, transformer)

        return result