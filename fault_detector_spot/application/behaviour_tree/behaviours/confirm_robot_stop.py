"""Wait for the shared execution owners to confirm an emergency stop."""

import time

import py_trees
from py_trees.common import Access, Status


class ConfirmRobotStop(py_trees.behaviour.Behaviour):
    def __init__(self, resources, name="ConfirmRobotStop", timeout_sec=10.0,
                 monotonic_clock=time.monotonic):
        super().__init__(name)
        self.resources = resources
        self.timeout_sec = timeout_sec
        self.clock = monotonic_clock
        self.node = None
        self.started = False
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("last_command", access=Access.READ)

    def setup(self, **kwargs):
        self.node = kwargs["node"]

    def initialise(self):
        self.started = False
        self.deadline = self.clock() + self.timeout_sec

    def update(self):
        command = self.blackboard.last_command
        try:
            if not self.started:
                self.resources.begin_stop_confirmation(self.node)
                self.started = True
            confirmed, detail = self.resources.poll_stop_confirmation()
        except Exception as exception:
            confirmed, detail = False, f"Stop confirmation unavailable: {exception}"
        command.execution_feedback_message = detail
        command.stop_confirmed = bool(confirmed)
        command.stop_unconfirmed = not confirmed and self.clock() >= self.deadline
        self.feedback_message = detail
        # A timeout is evidence of uncertainty, never permission to move on.
        return Status.SUCCESS if confirmed else Status.RUNNING
