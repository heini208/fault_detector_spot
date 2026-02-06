#!/usr/bin/env python3
from math import sin, cos, pi

import tf_transformations as tf
from builtin_interfaces.msg import Time
from fault_detector_spot.behaviour_tree.commands.command_ids import OrientationModes
from fault_detector_spot.behaviour_tree.commands.move_to_tag_command import MoveToTagCommand
from geometry_msgs.msg import PoseStamped, Quaternion
from synchros2.tf_listener_wrapper import TFListenerWrapper

# Precomputed constants for orientation modes
_YAW_90_SIN = sin(pi / 4)  # sin(45°)
_YAW_90_COS = cos(pi / 4)  # cos(45°)
_PITCH_45_SIN = sin(pi / 8)  # sin(22.5°)
_PITCH_45_COS = cos(pi / 8)  # cos(22.5°)


class ManipulatorToTagCommand(MoveToTagCommand):
    """
    Manipulator command to move to a tag + offset, handling orientation modes.
    Inherits position calculation from MoveToTagCommand but overrides orientation.
    """

    def __init__(
            self,
            command_id: str,
            stamp: Time,
            tag_pose: PoseStamped,
            tag_id: int,
            offset: PoseStamped = None,
            orientation_mode: str = "tag_orientation",
            target_frame: str = "body"
    ):
        # Initialize parent with goal_pose frame as the default target frame
        super().__init__(command_id, stamp, tag_pose, tag_id, offset, target_frame=target_frame)
        self.orientation_mode = orientation_mode

    def compute_goal_pose(self, transformer: TFListenerWrapper) -> PoseStamped:
        result = super().compute_goal_pose(transformer)

        # Apply orientation mode logic
        result = self._apply_orientation_mode(result, transformer)

        return result

    def _apply_orientation_mode(self, pose: PoseStamped, transformer: TFListenerWrapper) -> PoseStamped:
        """
        Adjusts the pose orientation based on the selected mode.
        'pose' currently contains the Tag's orientation (from parent).
        """
        if self.orientation_mode == OrientationModes.TAG_ORIENTATION:
            # Multiply the current Tag orientation (pose.pose.orientation) with the Offset orientation
            pose.pose.orientation = self._combine_orientations(pose.pose.orientation, self.offset.pose.orientation)

        elif self.orientation_mode == OrientationModes.CUSTOM_ORIENTATION:
            # Ignore Tag orientation. Use only the Offset orientation, rotated into target frame.
            pose.pose.orientation = self._get_rotated_offset_orientation(transformer)

        elif self.orientation_mode == OrientationModes.STRAIGHT:
            pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        elif self.orientation_mode == OrientationModes.LOOK_LEFT:
            pose.pose.orientation = Quaternion(
                x=0.0, y=0.0,
                z=_YAW_90_SIN,
                w=_YAW_90_COS
            )

        elif self.orientation_mode == OrientationModes.LOOK_RIGHT:
            pose.pose.orientation = Quaternion(
                x=0.0, y=0.0,
                z=-_YAW_90_SIN,
                w=_YAW_90_COS
            )

        elif self.orientation_mode == OrientationModes.LOOK_UP:
            pose.pose.orientation = Quaternion(
                x=0.0,
                y=-_PITCH_45_SIN,
                z=0.0,
                w=_PITCH_45_COS
            )

        elif self.orientation_mode == OrientationModes.LOOK_DOWN:
            pose.pose.orientation = Quaternion(
                x=0.0,
                y=_PITCH_45_SIN,
                z=0.0,
                w=_PITCH_45_COS
            )

        # If no match (or default fall-through), leave as is (Tag Orientation)
        return pose

    def _combine_orientations(self, q1_msg: Quaternion, q2_msg: Quaternion) -> Quaternion:
        """
        Combines two quaternions (q1 * q2).
        """
        q1 = [q1_msg.x, q1_msg.y, q1_msg.z, q1_msg.w]
        q2 = [q2_msg.x, q2_msg.y, q2_msg.z, q2_msg.w]

        q_combined = tf.quaternion_multiply(q1, q2)
        return Quaternion(x=q_combined[0], y=q_combined[1], z=q_combined[2], w=q_combined[3])

    def _get_rotated_offset_orientation(self, transformer: TFListenerWrapper) -> Quaternion:
        """
        Rotates the stored offset's orientation into the target frame.
        """
        q_offset = [
            self.offset.pose.orientation.x,
            self.offset.pose.orientation.y,
            self.offset.pose.orientation.z,
            self.offset.pose.orientation.w
        ]

        # Use helper from base MoveCommand
        rotated_q = self._rotate_quaternion_into_frame(
            q_offset,
            self.offset.header.frame_id,
            self.target_frame,
            transformer
        )

        return Quaternion(
            x=rotated_q[0], y=rotated_q[1], z=rotated_q[2], w=rotated_q[3]
        )