#!/usr/bin/env python3
from math import sin, cos, pi

import tf_transformations as tf
from builtin_interfaces.msg import Time
from fault_detector_spot.application.commanding.command_ids import (
    OrientationModes,
)
from fault_detector_spot.application.behaviour_tree.commands.move_to_tag_command import (
    MoveToTagCommand,
)
from fault_detector_spot.inspection.model.sensor_models import (
    BARE_HAND_MOTION_ID,
    sensor_probe_frame,
)
from geometry_msgs.msg import PoseStamped, Quaternion
from synchros2.tf_listener_wrapper import TFListenerWrapper

_YAW_90_SIN = sin(pi / 4)
_YAW_90_COS = cos(pi / 4)
_PITCH_45_SIN = sin(pi / 8)
_PITCH_45_COS = cos(pi / 8)


class ManipulatorToTagCommand(MoveToTagCommand):
    """
    Move the active probe point to a tag-relative target pose.
    """

    def __init__(
        self,
        command_id: str,
        stamp: Time,
        tag_pose: PoseStamped,
        tag_id: int,
        offset: PoseStamped = None,
        orientation_mode: str = "tag_orientation",
        target_frame: str = "body",
        motion_sensor_id: str = "",
    ):
        super().__init__(
            command_id,
            stamp,
            tag_pose,
            tag_id,
            offset,
            target_frame=target_frame,
        )
        self.orientation_mode = orientation_mode
        self.motion_sensor_id = motion_sensor_id.strip()

    def compute_goal_pose(
        self,
        transformer: TFListenerWrapper,
    ) -> PoseStamped:
        result = super().compute_goal_pose(transformer)
        result = self._apply_orientation_mode(result, transformer)
        return self._probe_target_to_hand_target(result, transformer)

    def _probe_target_to_hand_target(
        self,
        probe_target: PoseStamped,
        transformer: TFListenerWrapper,
    ) -> PoseStamped:
        if self.motion_sensor_id == BARE_HAND_MOTION_ID:
            return probe_target
        if not self.motion_sensor_id:
            raise ValueError(
                "Manipulator tag motion requires attachment geometry"
            )
        if transformer is None:
            raise RuntimeError(
                "Manipulator tag motion requires TF for sensor geometry"
            )

        probe_frame = sensor_probe_frame(self.motion_sensor_id)
        hand_to_probe = transformer.lookup_a_tform_b(
            "hand",
            probe_frame,
            timeout_sec=0.0,
        )
        target_to_probe = self._pose_matrix(probe_target.pose)
        hand_to_probe_matrix = self._transform_matrix(
            hand_to_probe.transform
        )
        target_to_hand = tf.concatenate_matrices(
            target_to_probe,
            tf.inverse_matrix(hand_to_probe_matrix),
        )
        translation = tf.translation_from_matrix(target_to_hand)
        quaternion = tf.quaternion_from_matrix(target_to_hand)

        probe_target.pose.position.x = float(translation[0])
        probe_target.pose.position.y = float(translation[1])
        probe_target.pose.position.z = float(translation[2])
        probe_target.pose.orientation = Quaternion(
            x=float(quaternion[0]),
            y=float(quaternion[1]),
            z=float(quaternion[2]),
            w=float(quaternion[3]),
        )
        return probe_target

    @staticmethod
    def _pose_matrix(pose):
        return tf.concatenate_matrices(
            tf.translation_matrix(
                [
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                ]
            ),
            tf.quaternion_matrix(
                [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ]
            ),
        )

    @staticmethod
    def _transform_matrix(transform):
        return tf.concatenate_matrices(
            tf.translation_matrix(
                [
                    transform.translation.x,
                    transform.translation.y,
                    transform.translation.z,
                ]
            ),
            tf.quaternion_matrix(
                [
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w,
                ]
            ),
        )

    def _apply_orientation_mode(
        self,
        pose: PoseStamped,
        transformer: TFListenerWrapper,
    ) -> PoseStamped:
        if self.orientation_mode in {
            OrientationModes.TAG_ORIENTATION,
            OrientationModes.CUSTOM_ORIENTATION,
        }:
            pose.pose.orientation = self._get_rotated_offset_orientation(
                transformer
            )
        elif self.orientation_mode == OrientationModes.STRAIGHT:
            pose.pose.orientation = Quaternion(
                x=0.0,
                y=0.0,
                z=0.0,
                w=1.0,
            )
        elif self.orientation_mode == OrientationModes.LOOK_LEFT:
            pose.pose.orientation = Quaternion(
                x=0.0,
                y=0.0,
                z=_YAW_90_SIN,
                w=_YAW_90_COS,
            )
        elif self.orientation_mode == OrientationModes.LOOK_RIGHT:
            pose.pose.orientation = Quaternion(
                x=0.0,
                y=0.0,
                z=-_YAW_90_SIN,
                w=_YAW_90_COS,
            )
        elif self.orientation_mode == OrientationModes.LOOK_UP:
            pose.pose.orientation = Quaternion(
                x=0.0,
                y=-_PITCH_45_SIN,
                z=0.0,
                w=_PITCH_45_COS,
            )
        elif self.orientation_mode == OrientationModes.LOOK_DOWN:
            pose.pose.orientation = Quaternion(
                x=0.0,
                y=_PITCH_45_SIN,
                z=0.0,
                w=_PITCH_45_COS,
            )
        return pose

    def _get_rotated_offset_orientation(
        self,
        transformer: TFListenerWrapper,
    ) -> Quaternion:
        q_offset = [
            self.offset.pose.orientation.x,
            self.offset.pose.orientation.y,
            self.offset.pose.orientation.z,
            self.offset.pose.orientation.w,
        ]
        rotated_q = self._rotate_quaternion_into_frame(
            q_offset,
            self.offset.header.frame_id,
            self.target_frame,
            transformer,
        )
        return Quaternion(
            x=rotated_q[0],
            y=rotated_q[1],
            z=rotated_q[2],
            w=rotated_q[3],
        )
