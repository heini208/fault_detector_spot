"""Shared live TF preparation for arm and base movement commands."""

import re

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped

from fault_detector_spot.application.commanding.command_ids import (
    OrientationModes,
    TagFrames,
)


class MovementGeometryUnavailable(RuntimeError):
    """Raised when live movement geometry is not available yet."""


class MovementGeometryResolver:
    """Resolve live TF-dependent movement geometry for executors."""

    def __init__(self, tf_listener):
        if tf_listener is None:
            raise ValueError(
                "MovementGeometryResolver requires a TF listener"
            )
        self.tf_listener = tf_listener

    def prepare_move_command(
        self,
        command,
        final_frame: str,
    ):
        """Prepare one command against the current TF state."""
        if not hasattr(command, "target_frame") or not hasattr(
            command,
            "offset",
        ):
            return command

        self.resolve_and_transform_offset_if_tag(command)

        target_frame = str(command.target_frame).strip()
        source_frame = str(command.offset.header.frame_id).strip()
        if not target_frame:
            raise ValueError("Movement target frame must not be empty")
        if not source_frame:
            raise ValueError("Movement offset frame must not be empty")

        if not self.can_transform(target_frame, source_frame):
            raise MovementGeometryUnavailable(
                f"Waiting for TF: {source_frame} -> {target_frame}"
            )

        normalized_final_frame = str(final_frame).strip()
        if not normalized_final_frame:
            raise ValueError("Movement execution frame must not be empty")

        if not self.can_transform(
            normalized_final_frame,
            target_frame,
        ):
            raise MovementGeometryUnavailable(
                "Waiting for TF: "
                f"{target_frame} -> {normalized_final_frame}"
            )

        tag_pose = getattr(command, "tag_pose", None)
        if tag_pose is not None:
            tag_frame = str(tag_pose.header.frame_id).strip()
            if not tag_frame:
                raise MovementGeometryUnavailable(
                    "Waiting for live tag pose frame"
                )
            if not self.can_transform(target_frame, tag_frame):
                raise MovementGeometryUnavailable(
                    f"Waiting for TF: {tag_frame} -> {target_frame}"
                )

        return command

    def resolve_and_transform_offset_if_tag(self, command):
        """Resolve Tag_N aliases and express their offset in target_frame."""
        source_frame = str(command.offset.header.frame_id).strip()
        if not self.is_tag_alias(source_frame):
            return command

        resolved = self.resolve_tag_alias(source_frame)
        if resolved == source_frame:
            raise MovementGeometryUnavailable(
                "Waiting for Tag_Transform"
            )

        target_frame = str(command.target_frame).strip()
        if not self.can_transform(target_frame, resolved):
            raise MovementGeometryUnavailable(
                f"Waiting for TF: {resolved} -> {target_frame}"
            )

        quaternion = [
            command.offset.pose.orientation.x,
            command.offset.pose.orientation.y,
            command.offset.pose.orientation.z,
            command.offset.pose.orientation.w,
        ]

        if (
            getattr(command, "orientation_mode", None)
            == OrientationModes.TAG_ORIENTATION
        ):
            rotated_orientation = quaternion
        else:
            rotated_orientation = command._rotate_only_yaw_into_frame(
                quaternion,
                resolved,
                target_frame,
                self.tf_listener,
            )

        offset = np.array(
            [
                command.offset.pose.position.x,
                command.offset.pose.position.y,
                command.offset.pose.position.z,
            ],
            dtype=float,
        )
        rotated_offset = command._rotate_vector_into_frame_yaw_only(
            offset,
            resolved,
            target_frame,
            self.tf_listener,
        )

        transformed = PoseStamped()
        transformed.header.frame_id = target_frame
        transformed.pose.position.x = float(rotated_offset[0])
        transformed.pose.position.y = float(rotated_offset[1])
        transformed.pose.position.z = float(rotated_offset[2])
        transformed.pose.orientation.x = rotated_orientation[0]
        transformed.pose.orientation.y = rotated_orientation[1]
        transformed.pose.orientation.z = rotated_orientation[2]
        transformed.pose.orientation.w = rotated_orientation[3]
        command.offset = transformed
        return command

    def resolve_tag_alias(self, frame_id: str) -> str:
        match = re.match(r"Tag[_:]?(\d+)$", str(frame_id))
        if not match:
            return frame_id

        tag_number = match.group(1)
        candidates = [
            f"{frame.value}{tag_number}"
            for frame in TagFrames
        ]
        for candidate in candidates:
            try:
                if self.tf_listener._tf_buffer.can_transform(
                    candidate,
                    candidate,
                    rclpy.time.Time(),
                ):
                    return candidate
            except Exception:
                continue
        return frame_id

    @staticmethod
    def is_tag_alias(frame_id: str) -> bool:
        return re.match(r"Tag[_:]?\d+$", str(frame_id)) is not None

    def can_transform(
        self,
        to_frame: str,
        from_frame: str,
    ) -> bool:
        if str(to_frame) == str(from_frame):
            return True
        try:
            return self.tf_listener._tf_buffer.can_transform(
                to_frame,
                from_frame,
                rclpy.time.Time(),
            )
        except Exception:
            return False


__all__ = [
    "MovementGeometryResolver",
    "MovementGeometryUnavailable",
]
