"""Behavior-tree adapter for geometric arm goal commands."""

import re

import numpy as np
import rclpy
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME
from geometry_msgs.msg import PoseStamped
from py_trees.common import Status

from fault_detector_spot.application.behaviour_tree.commands.move_command import (
    MoveCommand,
)
from fault_detector_spot.application.behaviour_tree.commands.move_to_tag_command import (
    MoveToTagCommand,
)
from fault_detector_spot.application.commanding.command_ids import (
    OrientationModes,
    TagFrames,
)
from fault_detector_spot.manipulation.behaviours.arm_movement_behaviour import (
    ArmMovementBehaviour,
)
from fault_detector_spot.manipulation.commands.manipulator_move_relative_command import (
    ManipulatorMoveRelativeCommand,
)
from fault_detector_spot.manipulation.commands.manipulator_to_tag_command import (
    ManipulatorToTagCommand,
)


class ArmGoalBehaviour(ArmMovementBehaviour):
    """Prepare one arm goal and dispatch it to ArmMovementExecutor."""

    def _prepare_operation(self):
        command = self._last_command()
        readiness = self._prepare_move_command(command)
        if readiness is not None:
            return readiness
        return None

    def _start_operation(self):
        command = self._last_command()

        if isinstance(command, ManipulatorMoveRelativeCommand):
            return self.executor.relative(command)

        if isinstance(command, ManipulatorToTagCommand):
            return self.executor.tag_probe(command)

        raise RuntimeError(
            "Expected manipulator relative or tag movement command, got "
            f"{type(command).__name__}"
        )

    def _last_command(self):
        if (
            not self.blackboard.exists("last_command")
            or self.blackboard.last_command is None
        ):
            raise RuntimeError("No command on blackboard")
        return self.blackboard.last_command

    def _prepare_move_command(self, command):
        if isinstance(command, MoveCommand):
            if not self._resolve_and_transform_offset_if_tag(command):
                return Status.RUNNING

            target_frame = command.target_frame
            source_frame = command.offset.header.frame_id

            if not self._can_transform(target_frame, source_frame):
                self.feedback_message = (
                    f"Waiting for TF: {source_frame} -> {target_frame}"
                )
                return Status.RUNNING

            if not self._can_transform(
                GRAV_ALIGNED_BODY_FRAME_NAME,
                target_frame,
            ):
                self.feedback_message = (
                    "Waiting for TF: "
                    f"{target_frame} -> "
                    f"{GRAV_ALIGNED_BODY_FRAME_NAME}"
                )
                return Status.RUNNING

        if isinstance(command, MoveToTagCommand):
            tag_frame = command.tag_pose.header.frame_id
            target_frame = command.target_frame
            if not self._can_transform(target_frame, tag_frame):
                self.feedback_message = (
                    f"Waiting for TF: {tag_frame} -> {target_frame}"
                )
                return Status.RUNNING

        return None

    def _resolve_and_transform_offset_if_tag(
        self,
        command: MoveCommand,
    ) -> bool:
        source_frame = command.offset.header.frame_id
        if not self._is_tag_alias(source_frame):
            return True

        resolved = self._resolve_tag_alias(source_frame)
        if resolved == source_frame:
            self.feedback_message = "Waiting for Tag_Transform"
            return False

        if not self._can_transform(resolved, command.target_frame):
            self.feedback_message = (
                f"Waiting for TF: {resolved} -> {command.target_frame}"
            )
            return False

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
            rotated_orientation = (
                command._rotate_only_yaw_into_frame(
                    quaternion,
                    resolved,
                    command.target_frame,
                    self.tf_listener,
                )
            )

        offset = np.array(
            [
                command.offset.pose.position.x,
                command.offset.pose.position.y,
                command.offset.pose.position.z,
            ]
        )
        rotated_offset = command._rotate_vector_into_frame_yaw_only(
            offset,
            resolved,
            command.target_frame,
            self.tf_listener,
        )

        transformed = PoseStamped()
        transformed.header.frame_id = command.target_frame
        transformed.pose.position.x = rotated_offset[0]
        transformed.pose.position.y = rotated_offset[1]
        transformed.pose.position.z = rotated_offset[2]
        transformed.pose.orientation.x = rotated_orientation[0]
        transformed.pose.orientation.y = rotated_orientation[1]
        transformed.pose.orientation.z = rotated_orientation[2]
        transformed.pose.orientation.w = rotated_orientation[3]
        command.offset = transformed
        return True

    def _resolve_tag_alias(self, frame_id: str) -> str:
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
    def _is_tag_alias(frame_id: str) -> bool:
        return re.match(r"Tag[_:]?\d+$", str(frame_id)) is not None

    def _can_transform(
        self,
        to_frame: str,
        from_frame: str,
    ) -> bool:
        try:
            return self.tf_listener._tf_buffer.can_transform(
                to_frame,
                from_frame,
                rclpy.time.Time(),
            )
        except Exception:
            return False


__all__ = ["ArmGoalBehaviour"]
