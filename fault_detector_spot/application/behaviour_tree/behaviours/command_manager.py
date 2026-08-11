"""Dispatch buffered commands with strictly increasing identities."""

import py_trees
from builtin_interfaces.msg import Time
from py_trees.common import Status

from fault_detector_spot.application.commanding.command_ids import (
    CommandID,
)


class CommandManager(py_trees.behaviour.Behaviour):
    """Promote buffered commands when the command tree is idle."""

    def __init__(self, name="CommandManager"):
        super().__init__(name)
        self.node = None
        self.blackboard = None
        self._last_dispatch_stamp_nanoseconds = -1

    def setup(self, **kwargs):
        self.node = kwargs.get("node")
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(
            key="command_buffer",
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.register_key(
            key="command_tree_status",
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.register_key(
            key="last_command",
            access=py_trees.common.Access.WRITE,
        )
        try:
            _ = self.blackboard.command_buffer
        except KeyError:
            self.blackboard.command_buffer = []
        try:
            _ = self.blackboard.command_tree_status
        except KeyError:
            self.blackboard.command_tree_status = None
        try:
            _ = self.blackboard.last_command
        except KeyError:
            self.blackboard.last_command = None
        self._last_dispatch_stamp_nanoseconds = -1
        return True

    def update(self):
        if not self.blackboard.command_buffer:
            return Status.SUCCESS

        if self.blackboard.command_tree_status == Status.FAILURE:
            failed_request_id = getattr(
                self.blackboard.last_command,
                "request_id",
                "",
            )
            if failed_request_id:
                self.blackboard.command_buffer[:] = [
                    command
                    for command in self.blackboard.command_buffer
                    if getattr(command, "request_id", "")
                    != failed_request_id
                ]
            if not self.blackboard.command_buffer:
                return Status.SUCCESS

        for buffered_command in list(
            self.blackboard.command_buffer
        ):
            if (
                buffered_command.command_id
                == CommandID.EMERGENCY_CANCEL
            ):
                self.blackboard.command_buffer.clear()
                buffered_command.stamp = self._next_dispatch_stamp()
                self.blackboard.last_command = buffered_command
                self.blackboard.command_tree_status = None
                return Status.SUCCESS

        tree_status = self.blackboard.command_tree_status
        if tree_status != Status.RUNNING:
            next_command = self.blackboard.command_buffer.pop(0)
            next_command.stamp = self._next_dispatch_stamp()
            self.blackboard.last_command = next_command
            self.blackboard.command_tree_status = None

        return Status.SUCCESS

    def _next_dispatch_stamp(self):
        now = self.node.get_clock().now()
        now_nanoseconds = getattr(now, "nanoseconds", None)
        if now_nanoseconds is None:
            now_message = now.to_msg()
            now_nanoseconds = (
                now_message.sec * 1_000_000_000
                + now_message.nanosec
            )

        dispatch_nanoseconds = max(
            0,
            int(now_nanoseconds),
            self._last_dispatch_stamp_nanoseconds + 1,
        )
        self._last_dispatch_stamp_nanoseconds = dispatch_nanoseconds

        stamp = Time()
        stamp.sec = dispatch_nanoseconds // 1_000_000_000
        stamp.nanosec = dispatch_nanoseconds % 1_000_000_000
        return stamp
