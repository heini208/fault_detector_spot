import py_trees
from py_trees.common import Status
from fault_detector_spot.behaviour_tree.command_ids import CommandID


class CommandManager(py_trees.behaviour.Behaviour):
    """
    Buffers incoming commands (from blackboard.command_buffer), ordered by stamp,
    and when the command tree is idle, pops the oldest and writes it to blackboard.last_command.
    If EMERGENCY_CANCEL appears, it's immediately promoted to last_command and buffer cleared.
    """

    def __init__(self, name="CommandManager"):
        super().__init__(name)

    def setup(self, **kwargs):
        self.node = kwargs.get("node")
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(
            key="command_buffer", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="command_tree_status", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="last_command", access=py_trees.common.Access.WRITE
        )
        # initialize if not present
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
        return True

    def update(self):
        if not self.blackboard.command_buffer:
            return Status.SUCCESS

        # if any buffered emergency cancel, fire it now
        for buffered_cmd in list(self.blackboard.command_buffer):
            if buffered_cmd.command_id == CommandID.EMERGENCY_CANCEL:
                self.blackboard.command_buffer.clear()
                self.blackboard.last_command = buffered_cmd
                return Status.SUCCESS

        # dispatch next when idle
        tree_status = self.blackboard.command_tree_status
        if self.blackboard.command_buffer and tree_status != Status.RUNNING:
            next_cmd = self.blackboard.command_buffer.pop(0)
            next_cmd.stamp = self.node.get_clock().now().to_msg()
            self.blackboard.last_command = next_cmd
            return Status.SUCCESS

        return Status.SUCCESS
