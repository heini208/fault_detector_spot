#!/usr/bin/env python3
from math import sin, cos, pi

import numpy as np

import tf_transformations as tf
from builtin_interfaces.msg import Time
from fault_detector_spot.behaviour_tree.commands.command_ids import OrientationModes
from geometry_msgs.msg import PoseStamped, Quaternion
from synchros2.tf_listener_wrapper import TFListenerWrapper
from .manipulator_move_command import ManipulatorMoveCommand

_YAW_90_SIN = sin(pi / 4)  # sin(45°)
_YAW_90_COS = cos(pi / 4)  # cos(45°)
_PITCH_45_SIN = sin(pi / 8)  # sin(22.5°)
_PITCH_45_COS = cos(pi / 8)  # cos(22.5°)


class ManipulatorTagCommand(ManipulatorMoveCommand):
    """
    Encapsulates a tag-based goal for the manipulator:
    - id: numeric tag identifier
    - goal_pose: PoseStamped from fiducial detection
    - offset: PoseStamped representing positional offsets

    Provides get_offset_pose() to combine them.
    """

    def __init__(
            self,
            command_id: str,
            stamp: Time,
            goal_pose: PoseStamped,  # in body frame
            tag_id: int,
            offset: PoseStamped = None,  # in target frame
            orientation_mode: str = "tag_orientation"
    ):
        super().__init__(command_id, stamp, goal_pose)

        self.tag_id = tag_id
        if offset is None:
            self.offset = PoseStamped()
            self.offset.header = goal_pose.header
            # zero position
            self.offset.pose.position.x = 0.0
            self.offset.pose.position.y = 0.0
            self.offset.pose.position.z = 0.0
            self.offset.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        else:
            self.offset = offset
        self.orientation_mode = orientation_mode

    def get_offset_pose(self, transformer: TFListenerWrapper = None) -> PoseStamped:
        """
        Returns a new PoseStamped combining the goal_pose and offset,
        with orientation adjusted according to orientation_mode.
        """
        result = PoseStamped()
        result.header = self.goal_pose.header

        # Step 1: rotate offset into goal frame
        rotated_offset = self._rotate_offset_into_goal_frame(transformer)
        # Step 2: apply rotated position offset
        result = self._apply_position_offset(rotated_offset, result)
        # Step 3: apply orientation offset
        result = self._apply_orientation_offset(result, transformer)
        return result

    def _apply_position_offset(self, offset_rotated: np.ndarray, pose: PoseStamped) -> PoseStamped:
        """
        Adds the rotated offset to the goal_pose position.
        """
        pose.pose.position.x = self.goal_pose.pose.position.x + offset_rotated[0]
        pose.pose.position.y = self.goal_pose.pose.position.y + offset_rotated[1]
        pose.pose.position.z = self.goal_pose.pose.position.z + offset_rotated[2]
        pose.pose.orientation = self.goal_pose.pose.orientation
        return pose

    def _apply_orientation_offset(self, pose: PoseStamped, transformer) -> PoseStamped:
        result = PoseStamped()
        result.header = pose.header
        result.pose.position = pose.pose.position

        if self.orientation_mode == OrientationModes.TAG_ORIENTATION:
            result.pose.orientation = self._get_tag_orientation_with_offset()
        elif self.orientation_mode == OrientationModes.CUSTOM_ORIENTATION:
            result.pose.orientation = self._apply_orientation_offset_in_tag_frame(transformer)
        elif self.orientation_mode == OrientationModes.STRAIGHT:
            result.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        elif self.orientation_mode == OrientationModes.LOOK_LEFT:
            result.pose.orientation = Quaternion(
                x=0.0, y=0.0,
                z=_YAW_90_SIN,
                w=_YAW_90_COS
            )

        elif self.orientation_mode == OrientationModes.LOOK_RIGHT:
            result.pose.orientation = Quaternion(
                x=0.0, y=0.0,
                z=-_YAW_90_SIN,
                w=_YAW_90_COS
            )

        elif self.orientation_mode == OrientationModes.LOOK_UP:
            result.pose.orientation = Quaternion(
                x=0.0,
                y=-_PITCH_45_SIN,
                z=0.0,
                w=_PITCH_45_COS
            )
        elif self.orientation_mode == OrientationModes.LOOK_DOWN:
            result.pose.orientation = Quaternion(
                x=0.0,
                y=_PITCH_45_SIN,
                z=0.0,
                w=_PITCH_45_COS
            )
        else:
            result.pose.orientation = pose.pose.orientation

        return result

    def _apply_orientation_offset_in_tag_frame(self, transformer):
        if transformer and self.offset.header.frame_id != self.goal_pose.header.frame_id:
            # rotate custom orientation from offset frame -> goal frame
            tf_msg = transformer.lookup_a_tform_b(
                self.goal_pose.header.frame_id,
                self.offset.header.frame_id,
                timeout_sec=2
            )
            q_rot = [
                tf_msg.transform.rotation.x,
                tf_msg.transform.rotation.y,
                tf_msg.transform.rotation.z,
                tf_msg.transform.rotation.w
            ]
            q_offset = [
                self.offset.pose.orientation.x,
                self.offset.pose.orientation.y,
                self.offset.pose.orientation.z,
                self.offset.pose.orientation.w
            ]
            q_rotated = tf.quaternion_multiply(q_rot, q_offset)
            return Quaternion(
                x=q_rotated[0], y=q_rotated[1], z=q_rotated[2], w=q_rotated[3]
            )
        else:
            # same frame, no transform needed
            return self.offset.pose.orientation

    def _get_tag_orientation_with_offset(self):
        q1 = self.goal_pose.pose.orientation
        q2 = self.offset.pose.orientation

        q1_list = [q1.x, q1.y, q1.z, q1.w]
        q2_list = [q2.x, q2.y, q2.z, q2.w]

        q_combined = tf.quaternion_multiply(q1_list, q2_list)

        return Quaternion(x=q_combined[0], y=q_combined[1], z=q_combined[2], w=q_combined[3])

    def _rotate_offset_into_goal_frame(self, transformer: TFListenerWrapper) -> np.ndarray:
        """
        Rotate the offset vector from its frame into the goal/tag frame axes.
        Returns a numpy 3-vector of the rotated offset.
        """
        offset_vec = np.array([
            self.offset.pose.position.x,
            self.offset.pose.position.y,
            self.offset.pose.position.z
        ])

        if transformer and self.offset.header.frame_id != self.goal_pose.header.frame_id:
            # lookup rotation from offset frame -> goal frame
            tf_msg = transformer.lookup_a_tform_b(self.goal_pose.header.frame_id,
                                                  self.offset.header.frame_id,
                                                  timeout_sec=2)
            q_rot = [
                tf_msg.transform.rotation.x,
                tf_msg.transform.rotation.y,
                tf_msg.transform.rotation.z,
                tf_msg.transform.rotation.w
            ]
            R = tf.quaternion_matrix(q_rot)[:3, :3]
        else:
            R = np.eye(3)

        offset_rotated = R.dot(offset_vec)
        return offset_rotated

    def get_as_manipulator_move_command_with_offset(self, transformer: TFListenerWrapper) -> ManipulatorMoveCommand:
        """
        Returns a ManipulatorMoveCommand with the same goal pose and offset applied.
        """
        return ManipulatorMoveCommand(
            command_id=self.command_id,
            stamp=self.stamp,
            goal_pose=self.get_offset_pose(transformer),
        )