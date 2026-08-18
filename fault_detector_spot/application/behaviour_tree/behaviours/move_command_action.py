import re

import numpy as np
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME

import rclpy
from fault_detector_spot.application.commanding.command_ids import (
    OrientationModes,
    TagFrames,
)
from fault_detector_spot.application.behaviour_tree.behaviours.spot_action import ActionClientBehaviour
from fault_detector_spot.application.behaviour_tree.commands.move_command import MoveCommand
from fault_detector_spot.application.behaviour_tree.commands.move_to_tag_command import (
    MoveToTagCommand,
)
from geometry_msgs.msg import PoseStamped
from py_trees.common import Status, Access
from spot_msgs.action import RobotCommand
from synchros2.action_client import ActionClientWrapper
from synchros2.tf_listener_wrapper import TFListenerWrapper
from synchros2.utilities import namespace_with


class MoveCommandAction(ActionClientBehaviour):
    """
    Intermediate base class for actions that execute a MoveCommand.

    It overrides update() to ensure all necessary TF frames are available
    BEFORE delegating to the ActionClientBehaviour lifecycle.
    """

    def __init__(self, name: str, robot_name: str = ""):
        super().__init__(name)
        self.tf_listener: TFListenerWrapper = None
        self.blackboard = self.attach_blackboard_client()
        self.robot_name = robot_name
        self.blackboard.register_key(key="last_command", access=Access.READ)

    def setup(self, **kwargs):
        super().setup(**kwargs)

    def _init_client(self) -> bool:
        action_ns = namespace_with(self.robot_name, "robot_command")
        if self._client is None:
            self._client = ActionClientWrapper(RobotCommand, action_ns, self.node)
        if self.tf_listener is None:
            self.tf_listener = TFListenerWrapper(self.node)
        self.initialized = True
        return True

    def _phase_send_goal(self) -> Status | None:
        try:
            return self._phase_send_goal_impl()
        except Exception as exception:
            self.feedback_message = f"Move goal preparation failed: {exception}"
            self.logger.error(f"[{self.name}] {self.feedback_message}")
            self._reset_state()
            return Status.FAILURE

    def _phase_send_goal_impl(self) -> Status | None:
        if self.send_goal_future is None:
            cmd = self._get_last_command()
            if cmd is None:
                return Status.FAILURE
            if isinstance(cmd, MoveCommand):
                ok = self._resolve_and_transform_offset_if_tag(cmd)
                if not ok:
                    return Status.RUNNING

                target_frame = cmd.target_frame
                source_frame = cmd.offset.header.frame_id
                final_frame = GRAV_ALIGNED_BODY_FRAME_NAME

                if not self._can_transform(target_frame, source_frame):
                    self.feedback_message = f"Waiting for TF: {source_frame} -> {target_frame}"
                    return Status.RUNNING
                if not self._can_transform(final_frame, target_frame):
                    self.feedback_message = f"Waiting for TF: {target_frame} -> {final_frame}"
                    return Status.RUNNING

            if isinstance(cmd, MoveToTagCommand):
                tag_frame = cmd.tag_pose.header.frame_id
                target_frame = cmd.target_frame
                if not self._can_transform(target_frame, tag_frame):
                    self.feedback_message = f"Waiting for TF: {tag_frame} -> {target_frame}"
                    return Status.RUNNING

        return super()._phase_send_goal()

    def _get_last_command(self):
        if not self.blackboard.exists("last_command") or self.blackboard.last_command is None:
            self.feedback_message = "No command on blackboard"
            return None
        return self.blackboard.last_command

    def _resolve_and_transform_offset_if_tag(self, cmd: MoveCommand) -> bool:
        source_frame = cmd.offset.header.frame_id
        if self._is_tag_alias(source_frame):
            resolved = self._resolve_tag_alias(source_frame)
            if resolved == source_frame:
                self.feedback_message = "Waiting for Tag_Transform"
                return False
            if not self._can_transform(resolved, cmd.target_frame):
                self.feedback_message = f"Waiting for TF: {resolved} -> {cmd.target_frame}"
                return False

            quat = [
                cmd.offset.pose.orientation.x,
                cmd.offset.pose.orientation.y,
                cmd.offset.pose.orientation.z,
                cmd.offset.pose.orientation.w,
            ]

            if (
                getattr(cmd, "orientation_mode", None)
                == OrientationModes.TAG_ORIENTATION
            ):
                rotated_orientation = quat
            else:
                rotated_orientation = cmd._rotate_only_yaw_into_frame(
                    quat,
                    resolved,
                    cmd.target_frame,
                    self.tf_listener,
                )
            offset_vec = np.array(
                [
                    cmd.offset.pose.position.x,
                    cmd.offset.pose.position.y,
                    cmd.offset.pose.position.z,
                ]
            )
            rotated_vector = cmd._rotate_vector_into_frame_yaw_only(
                offset_vec,
                resolved,
                cmd.target_frame,
                self.tf_listener,
            )

            new_offset = PoseStamped()
            new_offset.header.frame_id = cmd.target_frame
            new_offset.pose.position.x = rotated_vector[0]
            new_offset.pose.position.y = rotated_vector[1]
            new_offset.pose.position.z = rotated_vector[2]
            new_offset.pose.orientation.x = rotated_orientation[0]
            new_offset.pose.orientation.y = rotated_orientation[1]
            new_offset.pose.orientation.z = rotated_orientation[2]
            new_offset.pose.orientation.w = rotated_orientation[3]
            cmd.offset = new_offset
        return True

    def _resolve_tag_alias(self, frame_id: str) -> str:
        """Given Tag_X etc, try real tag frames. Returns best or original name if none is found."""
        match = re.match(r"Tag[_:]?(\d+)$", str(frame_id))
        if match:
            tagnum = match.group(1)
            candidates = [f"{frame.value}{tagnum}" for frame in TagFrames]
            for cand in candidates:
                try:
                    if self.tf_listener._tf_buffer.can_transform(cand, cand, rclpy.time.Time()):
                        return cand
                except Exception:
                    continue
        return frame_id

    def _is_tag_alias(self, frame_id: str) -> bool:
        return re.match(r"Tag[_:]?\d+$", str(frame_id)) is not None

    def _can_transform(self, to_frame: str, from_frame: str) -> bool:
        try:
            return self.tf_listener._tf_buffer.can_transform(to_frame, from_frame, rclpy.time.Time())
        except Exception:
            return False
