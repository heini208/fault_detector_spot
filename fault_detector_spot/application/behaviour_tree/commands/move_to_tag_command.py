#!/usr/bin/env python3
from copy import deepcopy

import numpy as np
import tf2_geometry_msgs
from builtin_interfaces.msg import Time
from fault_detector_spot.application.behaviour_tree.commands.move_command import (
    MoveCommand,
)
from geometry_msgs.msg import PoseStamped
from synchros2.tf_listener_wrapper import TFListenerWrapper


class MoveToTagCommand(MoveCommand):
    """Movement to a tag-relative target with an added offset."""

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

    def transform_tag_to_target_frame(
        self,
        transformer: TFListenerWrapper,
    ) -> PoseStamped:
        source_frame = self.tag_pose.header.frame_id.strip()
        target_frame = self.target_frame.strip()
        if not source_frame:
            raise ValueError("Tag pose frame must not be empty")
        if not target_frame:
            raise ValueError("Tag target frame must not be empty")
        if source_frame == target_frame:
            return deepcopy(self.tag_pose)
        if transformer is None:
            raise RuntimeError(
                "Tag movement between different frames requires TF"
            )
        tf_to_target = transformer.lookup_a_tform_b(
            target_frame,
            source_frame,
            timeout_sec=0.0,
        )
        return tf2_geometry_msgs.do_transform_pose_stamped(
            transform=tf_to_target,
            pose=self.tag_pose,
        )

    def add_offset_to_tag_pose(
        self,
        tag_in_target: PoseStamped,
        transformer: TFListenerWrapper,
    ) -> PoseStamped:
        offset_vec = np.array(
            [
                self.offset.pose.position.x,
                self.offset.pose.position.y,
                self.offset.pose.position.z,
            ],
            dtype=float,
        )
        rotated_offset = self._rotate_vector_into_frame(
            offset_vec,
            self.offset.header.frame_id,
            self.target_frame,
            transformer,
        )

        result = PoseStamped()
        result.header.frame_id = self.target_frame
        result.header.stamp = self.stamp
        result.pose.position.x = (
            tag_in_target.pose.position.x + float(rotated_offset[0])
        )
        result.pose.position.y = (
            tag_in_target.pose.position.y + float(rotated_offset[1])
        )
        result.pose.position.z = (
            tag_in_target.pose.position.z + float(rotated_offset[2])
        )
        result.pose.orientation = tag_in_target.pose.orientation
        return result

    def compute_goal_pose(
        self,
        transformer: TFListenerWrapper,
    ) -> PoseStamped:
        tag_in_target = self.transform_tag_to_target_frame(transformer)
        return self.add_offset_to_tag_pose(tag_in_target, transformer)
