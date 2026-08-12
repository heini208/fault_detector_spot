import time

import py_trees
from py_trees.common import Status


class WaitForDuration(py_trees.behaviour.Behaviour):
    """Wait for the duration attached to the active command request."""

    def __init__(
        self,
        name: str = "WaitForDuration",
        monotonic_clock=time.monotonic,
    ):
        super().__init__(name)
        self._clock = monotonic_clock
        self._request_id = ""
        self._deadline = None

    def setup(self, **kwargs) -> bool:
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(
            key="last_command",
            access=py_trees.common.Access.READ,
        )
        return True

    def initialise(self) -> None:
        command = self.blackboard.last_command
        request_id = getattr(command, "request_id", "")
        if request_id == self._request_id and self._deadline is not None:
            return
        duration = float(getattr(command, "duration", 0.0) or 0.0)
        self._request_id = request_id
        self._deadline = self._clock() + max(0.0, duration)

    def update(self) -> Status:
        if self._deadline is None or self._clock() >= self._deadline:
            return Status.SUCCESS
        return Status.RUNNING
