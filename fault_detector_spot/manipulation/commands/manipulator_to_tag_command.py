#!/usr/bin/env python3
from math import cos, pi, sin

from builtin_interfaces.msg import Time
from fault_detector_spot.application.commanding.command_ids import (
    OrientationModes,
)
from fault_detector_spot.application.behaviour_tree.commands.move_to_tag_command import (
    MoveToTagCommand,
)
from fault_detector_spot.inspection.geometry.rotation import (
    multiply_quaternions,
)
from fault_detector_spot.inspection.model.models import QuaternionData
from fault_detector_spot.inspection.model.sensor_models import (
    BARE_HAND_MOTION_ID,
    sensor_probe_frame,
)
from fault_detector_spot.shared.geometry.transforms import (
    compose_poses,
    inverse_pose,
    pose_data_to_pose,
)
from fault_detector_spot.shared.ros.tf_transforms import transform_to_pose_data
from geometry_msgs.msg import PoseStamped, Quaternion
from synchros2.tf_listener_wrapper import TFListenerWrapper


_YAW_90_SIN = sin(pi / 4)
_YAW_90_COS = cos(pi / 4)
_PITCH_45_SIN = sin(pi / 8)
_PITCH_45_COS = cos(pi / 8)


class ManipulatorToTagCommand(MoveToTagCommand):
    """Move the active probe point to a tag-relative target pose."""

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
        hand_to_probe_pose = pose_data_to_pose(
            transform_to_pose_data(hand_to_probe)
        )
        probe_target.pose = compose_poses(
            probe_target.pose,
            inverse_pose(hand_to_probe_pose),
        )
        return probe_target

    def _apply_orientation_mode(
        self,
        pose: PoseStamped,
        transformer: TFListenerWrapper,
    ) -> PoseStamped:
        if self.orientation_mode == OrientationModes.TAG_ORIENTATION:
            pose.pose.orientation = self._combine_orientations(
                pose.pose.orientation,
                self.offset.pose.orientation,
            )
        elif self.orientation_mode == OrientationModes.CUSTOM_ORIENTATION:
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

    @staticmethod
    def _combine_orientations(
        q1_msg: Quaternion,
        q2_msg: Quaternion,
    ) -> Quaternion:
        first = QuaternionData(
            x=float(q1_msg.x),
            y=float(q1_msg.y),
            z=float(q1_msg.z),
            w=float(q1_msg.w),
        )
        second = QuaternionData(
            x=float(q2_msg.x),
            y=float(q2_msg.y),
            z=float(q2_msg.z),
            w=float(q2_msg.w),
        )
        combined = multiply_quaternions(first, second)
        return Quaternion(
            x=combined.x,
            y=combined.y,
            z=combined.z,
            w=combined.w,
        )

    def _get_rotated_offset_orientation(
        self,
        transformer: TFListenerWrapper,
    ) -> Quaternion:
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
        return Quaternion(
            x=rotated_q[0],
            y=rotated_q[1],
            z=rotated_q[2],
            w=rotated_q[3],
        )
