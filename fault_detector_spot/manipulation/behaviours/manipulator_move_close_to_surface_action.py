"""Behavior-tree client for the close-surface ROS action server."""

from threading import RLock
import time
from typing import Optional, Tuple

from fault_detector_msgs.action import MoveCloseToSurface
from fault_detector_spot.application.behaviour_tree.behaviours.spot_action import (
    WorkflowActionBehaviour,
)
from fault_detector_spot.manipulation.commands.move_close_to_surface_command import (
    MoveCloseToSurfaceCommand,
)
from fault_detector_spot.manipulation.move_close_to_surface_interface import (
    MOVE_CLOSE_TO_SURFACE_ACTION,
)
from py_trees.common import Access, Status


class ManipulatorMoveCloseToSurfaceAction(WorkflowActionBehaviour):
    """Delegate close-surface execution to its dedicated ROS process."""

    def __init__(
        self,
        name: str = "MoveCloseToSurface",
        action_name: str = MOVE_CLOSE_TO_SURFACE_ACTION,
        goal_response_timeout_sec: float = 2.0,
        result_timeout_sec: float = 600.0,
        monotonic_clock=time.monotonic,
    ):
        super().__init__(
            name,
            action_type=MoveCloseToSurface,
            action_name=action_name,
            goal_response_timeout_sec=goal_response_timeout_sec,
            result_timeout_sec=result_timeout_sec,
            monotonic_clock=monotonic_clock,
        )
        self._lock = RLock()
        self._command = None
        self._feedback = ""
        self.blackboard.register_key(
            "command_failure_request_id",
            access=Access.WRITE,
        )
        self.blackboard.register_key(
            "command_failure_detail",
            access=Access.WRITE,
        )

    def initialise(self):
        super().initialise()
        self.feedback_message = "Waiting for close-surface action server"

    def _before_update(self) -> Optional[Status]:
        command = self._current_command()
        if self._command is None:
            self._command = command
            self.blackboard.command_failure_request_id = command.request_id
            self.blackboard.command_failure_detail = ""
        elif command is not self._command:
            return self._fail(
                "Close-surface command changed during execution"
            )

        with self._lock:
            feedback = self._feedback
        if feedback:
            self.feedback_message = feedback
        return None

    def _build_goal(self) -> MoveCloseToSurface.Goal:
        command = self._command
        if command is None:
            raise RuntimeError("Close-surface command is unavailable")
        goal = MoveCloseToSurface.Goal()
        goal.request_id = command.request_id
        goal.target_surface_distance_m = command.target_surface_distance_m
        goal.aligned_preapproach_distance_m = (
            command.aligned_preapproach_distance_m
        )
        return goal

    def _send_goal(self, goal):
        return self._client.send_goal_async(
            goal,
            feedback_callback=self._receive_feedback,
        )

    def _interpret_result(self, result) -> Tuple[Status, str]:
        detail = result.detail.strip()
        if result.success:
            return (
                Status.SUCCESS,
                detail or "Close-surface movement completed",
            )
        return Status.FAILURE, detail or "Close-surface movement failed"

    def _on_failure(self, detail: str) -> None:
        request_id = getattr(self._command, "request_id", "")
        self.blackboard.command_failure_request_id = request_id
        self.blackboard.command_failure_detail = detail

    def _reset_subclass_state(self) -> None:
        self._command = None
        with self._lock:
            self._feedback = ""

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


__all__ = [
    "MOVE_CLOSE_TO_SURFACE_ACTION",
    "ManipulatorMoveCloseToSurfaceAction",
]
