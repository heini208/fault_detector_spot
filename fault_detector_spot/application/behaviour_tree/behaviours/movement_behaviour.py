"""Common py_trees adapter for movement executors."""

import re

import numpy as np
import py_trees
import rclpy
import synchros2.scope as ros_scope
from geometry_msgs.msg import PoseStamped
from py_trees.common import Access, Status

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


class MovementBehaviour(py_trees.behaviour.Behaviour):
    """Translate behavior-tree ticks into movement executor calls."""

    RUNNING_OUTCOME = None
    SUCCESS_OUTCOME = None

    def __init__(self, name: str):
        super().__init__(name)
        self.node = None
        self.executor = None
        self.tf_listener = None
        self._started = False

        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(
            "last_command",
            access=Access.READ,
        )
        self.blackboard.register_key(
            "command_failure_request_id",
            access=Access.WRITE,
        )
        self.blackboard.register_key(
            "command_failure_detail",
            access=Access.WRITE,
        )

    def setup(self, **kwargs):
        self.node = kwargs.get("node") or ros_scope.node()
        if self.node is None:
            raise RuntimeError(
                f"{self.__class__.__name__} requires a ROS node"
            )

    def initialise(self):
        self._started = False
        self._clear_failure_detail()

    def update(self) -> Status:
        try:
            self._ensure_executor()

            if self._started:
                update = self.executor.poll()
            else:
                preparation = self._prepare_operation()
                if preparation is Status.RUNNING:
                    return Status.RUNNING
                if preparation is Status.FAILURE:
                    return self._fail(
                        self.feedback_message
                        or "Movement preparation failed"
                    )
                update = self._start_operation()

            self.feedback_message = update.detail
            if update.outcome is self.RUNNING_OUTCOME:
                self._started = True
                return Status.RUNNING

            self._started = False
            if update.outcome is self.SUCCESS_OUTCOME:
                return Status.SUCCESS

            return self._fail(update.detail)
        except Exception as exception:
            return self._fail(
                f"{self.__class__.__name__} failed: {exception}"
            )

    def terminate(self, new_status: Status):
        if new_status is Status.INVALID and self._started:
            self.executor.cancel()
        self._started = False

    def shutdown(self):
        if self._started and self.executor is not None:
            self.executor.cancel()
        self._started = False

    def _prepare_operation(self):
        return None

    def _ensure_executor(self) -> None:
        raise NotImplementedError

    def _start_operation(self):
        raise NotImplementedError

    def _last_command(self):
        if (
            not self.blackboard.exists("last_command")
            or self.blackboard.last_command is None
        ):
            raise RuntimeError("No command on blackboard")
        return self.blackboard.last_command

    def _prepare_move_command(
        self,
        command,
        final_frame: str,
    ):
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

            if not self._can_transform(final_frame, target_frame):
                self.feedback_message = (
                    f"Waiting for TF: {target_frame} -> {final_frame}"
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
            rotated_orientation = command._rotate_only_yaw_into_frame(
                quaternion,
                resolved,
                command.target_frame,
                self.tf_listener,
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

    def _fail(self, detail: str) -> Status:
        if self._started and self.executor is not None:
            self.executor.cancel()
        self._started = False

        normalized = str(detail).strip() or "Movement failed"
        self.blackboard.command_failure_request_id = (
            self._current_request_id()
        )
        self.blackboard.command_failure_detail = normalized
        self.feedback_message = normalized
        return Status.FAILURE

    def _clear_failure_detail(self) -> None:
        self.blackboard.command_failure_request_id = (
            self._current_request_id()
        )
        self.blackboard.command_failure_detail = ""

    def _current_request_id(self) -> str:
        try:
            command = self.blackboard.last_command
        except (AttributeError, KeyError):
            return ""
        return str(getattr(command, "request_id", "") or "")


__all__ = ["MovementBehaviour"]
