"""Thin behavior-tree adapter for ArmMovementExecutor.prepare()."""

import py_trees
from py_trees.common import Access, Status
import synchros2.scope as ros_scope

from fault_detector_spot.manipulation.arm_movement_executor import (
    ArmMovementOutcome,
)


class ReadyArmActionSimple(py_trees.behaviour.Behaviour):
    """Expose the executor's prepare operation to the command tree."""

    def __init__(
        self,
        name: str = "ReadyArmAction",
        robot_name: str = "",
        robot_command_resources=None,
    ):
        super().__init__(name)
        self.robot_name = robot_name
        self.robot_command_resources = robot_command_resources
        self.arm_movement_executor = None
        self.node = None
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
                "ReadyArmActionSimple requires a ROS node"
            )

    def initialise(self):
        self._started = False
        self._clear_failure_detail()

    def update(self) -> Status:
        try:
            self._ensure_executor()
            if self._started:
                update = self.arm_movement_executor.poll()
            else:
                update = self.arm_movement_executor.prepare()

            self.feedback_message = update.detail
            if update.outcome is ArmMovementOutcome.RUNNING:
                self._started = True
                return Status.RUNNING

            self._started = False
            if update.outcome is ArmMovementOutcome.SUCCESS:
                return Status.SUCCESS
            return self._fail(update.detail)
        except Exception as exception:
            return self._fail(
                f"Ready arm failed: {exception}"
            )

    def terminate(self, new_status: Status):
        if new_status is Status.INVALID and self._started:
            self.arm_movement_executor.cancel()
        self._started = False

    def shutdown(self):
        if (
            self._started
            and self.arm_movement_executor is not None
        ):
            self.arm_movement_executor.cancel()
        self._started = False

    def _ensure_executor(self) -> None:
        if self.arm_movement_executor is not None:
            return
        if self.robot_command_resources is None:
            raise RuntimeError(
                "ReadyArmAction requires shared robot command resources"
            )
        self.arm_movement_executor = (
            self.robot_command_resources.get_arm_movement_executor(
                self.node,
                robot_name=self.robot_name,
            )
        )

    def _fail(self, detail: str) -> Status:
        if (
            self._started
            and self.arm_movement_executor is not None
        ):
            self.arm_movement_executor.cancel()
        self._started = False
        normalized = str(detail).strip() or "Ready arm failed"
        self._on_failure(normalized)
        self.feedback_message = normalized
        return Status.FAILURE

    def _clear_failure_detail(self) -> None:
        self.blackboard.command_failure_request_id = (
            self._current_request_id()
        )
        self.blackboard.command_failure_detail = ""

    def _on_failure(self, detail: str) -> None:
        self.blackboard.command_failure_request_id = (
            self._current_request_id()
        )
        self.blackboard.command_failure_detail = detail

    def _current_request_id(self) -> str:
        try:
            command = self.blackboard.last_command
        except (AttributeError, KeyError):
            return ""
        return str(getattr(command, "request_id", "") or "")
