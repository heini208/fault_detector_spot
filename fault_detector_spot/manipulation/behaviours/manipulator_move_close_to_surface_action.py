"""Behavior-tree client for the close-surface ROS action server."""

from threading import RLock

import py_trees
from fault_detector_msgs.action import MoveCloseToSurface
from py_trees.common import Access, Status
from rclpy.action import ActionClient

from fault_detector_spot.manipulation.commands.move_close_to_surface_command import (
    MoveCloseToSurfaceCommand,
)
from fault_detector_spot.manipulation.move_close_to_surface_interface import (
    MOVE_CLOSE_TO_SURFACE_ACTION,
)


class ManipulatorMoveCloseToSurfaceAction(py_trees.behaviour.Behaviour):
    """Delegate close-surface execution to its dedicated ROS process."""

    def __init__(
        self,
        name: str = "MoveCloseToSurface",
        action_name: str = MOVE_CLOSE_TO_SURFACE_ACTION,
    ):
        super().__init__(name)
        self.action_name = action_name
        self.node = None
        self._client = None
        self._lock = RLock()
        self._command = None
        self._send_goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._feedback = ""
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("last_command", access=Access.READ)
        self.blackboard.register_key(
            "command_failure_request_id",
            access=Access.WRITE,
        )
        self.blackboard.register_key(
            "command_failure_detail",
            access=Access.WRITE,
        )

    def setup(self, **kwargs):
        self.node = kwargs.get("node")
        if self.node is None:
            raise RuntimeError(f"{self.name}: no ROS node provided")
        self._client = ActionClient(
            self.node,
            MoveCloseToSurface,
            self.action_name,
        )
        return True

    def initialise(self):
        self._reset_request_state()
        self.feedback_message = "Waiting for close-surface action server"

    def update(self) -> Status:
        try:
            command = self._current_command()
        except Exception as exception:
            return self._fail(str(exception))

        if self._command is None:
            self._command = command
            self.blackboard.command_failure_request_id = command.request_id
            self.blackboard.command_failure_detail = ""
        elif command is not self._command:
            return self._fail(
                "Close-surface command changed during execution"
            )

        if self._send_goal_future is None:
            if not self._client.server_is_ready():
                return self._fail(
                    "Close-surface action server is unavailable"
                )
            goal = MoveCloseToSurface.Goal()
            goal.request_id = command.request_id
            goal.target_surface_distance_m = (
                command.target_surface_distance_m
            )
            self._send_goal_future = self._client.send_goal_async(
                goal,
                feedback_callback=self._receive_feedback,
            )
            self.feedback_message = "Close-surface goal sent"
            return Status.RUNNING

        if self._goal_handle is None:
            if not self._send_goal_future.done():
                return Status.RUNNING
            try:
                self._goal_handle = self._send_goal_future.result()
            except Exception as exception:
                return self._fail(
                    f"Close-surface goal submission failed: {exception}"
                )
            if not self._goal_handle.accepted:
                return self._fail("Close-surface goal was rejected")
            self._result_future = self._goal_handle.get_result_async()

        with self._lock:
            feedback = self._feedback
        if feedback:
            self.feedback_message = feedback

        if self._result_future is None or not self._result_future.done():
            return Status.RUNNING

        try:
            result = self._result_future.result().result
        except Exception as exception:
            return self._fail(
                f"Close-surface action failed: {exception}"
            )
        detail = result.detail.strip()
        if not result.success:
            return self._fail(detail or "Close-surface movement failed")
        self.feedback_message = detail or "Close-surface movement completed"
        return Status.SUCCESS

    def terminate(self, new_status: Status):
        if new_status == Status.INVALID:
            self._cancel_goal()
        self._reset_request_state()

    def shutdown(self):
        self._cancel_goal()
        if self._client is not None:
            self._client.destroy()
            self._client = None

    def _current_command(self) -> MoveCloseToSurfaceCommand:
        if not self.blackboard.exists("last_command"):
            raise RuntimeError("No command is available on the blackboard")
        command = self.blackboard.last_command
        if not isinstance(command, MoveCloseToSurfaceCommand):
            raise RuntimeError(
                "Expected MoveCloseToSurfaceCommand, got "
                f"{type(command).__name__}"
            )
        return command

    def _receive_feedback(self, message) -> None:
        feedback = message.feedback
        detail = feedback.detail.strip()
        phase = feedback.phase.strip()
        with self._lock:
            self._feedback = detail or phase

    def _cancel_goal(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            return
        future = self._send_goal_future
        if future is None or future.done():
            return

        def cancel_when_accepted(done_future):
            try:
                goal_handle = done_future.result()
            except Exception:
                return
            if goal_handle is not None and goal_handle.accepted:
                goal_handle.cancel_goal_async()

        future.add_done_callback(cancel_when_accepted)

    def _fail(self, detail: str) -> Status:
        normalized = str(detail).strip() or "Close-surface movement failed"
        request_id = getattr(self._command, "request_id", "")
        self.blackboard.command_failure_request_id = request_id
        self.blackboard.command_failure_detail = normalized
        self.feedback_message = normalized
        return Status.FAILURE

    def _reset_request_state(self) -> None:
        self._command = None
        self._send_goal_future = None
        self._goal_handle = None
        self._result_future = None
        with self._lock:
            self._feedback = ""


__all__ = [
    "MOVE_CLOSE_TO_SURFACE_ACTION",
    "ManipulatorMoveCloseToSurfaceAction",
]
