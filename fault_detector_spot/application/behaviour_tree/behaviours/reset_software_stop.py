"""Reset software admission only after fresh owner stop confirmation."""

import py_trees
from py_trees.common import Access, Status


class ResetSoftwareStop(py_trees.behaviour.Behaviour):
    def __init__(self, resources, status_ledger, name="ResetSoftwareStop"):
        super().__init__(name)
        self.resources = resources
        self.status_ledger = status_ledger
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("last_command", access=Access.READ)
        self.blackboard.register_key("command_buffer", access=Access.WRITE)
        self.blackboard.register_key("estop_flag", access=Access.WRITE)

    def initialise(self):
        self.started = False

    def update(self):
        command = self.blackboard.last_command
        if not self.started:
            if not self.resources.begin_reset_confirmation():
                command.execution_feedback_message = "Reset requires an already confirmed software stop"
                return Status.FAILURE
            self.started = True
        confirmed, detail = self.resources.poll_stop_confirmation()
        command.execution_feedback_message = detail
        if not confirmed:
            return Status.RUNNING
        reset, detail = self.status_ledger.reset_stop(
            command.request_id, self.resources.reset_software_stop,
            clear_admission=self._clear_tree_admission,
        )
        command.execution_feedback_message = detail
        if not reset:
            return Status.FAILURE
        return Status.SUCCESS

    def _clear_tree_admission(self):
        # Called inside the same generation-checked transition that clears the
        # physical submission gate. Never erase a later stop after it returns.
        self.blackboard.command_buffer.clear()
        self.blackboard.estop_flag = False
