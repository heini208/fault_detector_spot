"""Common py_trees adapter for movement executors."""

import py_trees
import synchros2.scope as ros_scope
from py_trees.common import Access, Status


class MovementBehaviour(py_trees.behaviour.Behaviour):
    """Translate behavior-tree ticks into movement executor calls."""

    RUNNING_OUTCOME = None
    SUCCESS_OUTCOME = None

    def __init__(self, name: str):
        super().__init__(name)
        self.node = None
        self.executor = None
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
