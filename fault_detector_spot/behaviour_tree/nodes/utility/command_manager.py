import bisect
import py_trees
from py_trees.common import Status
from fault_detector_spot.behaviour_tree.command_ids import CommandID


class CommandManager(py_trees.behaviour.Behaviour):
    """
    Buffers incoming commands (from blackboard.last_command_received), ordered by stamp,
    and when the command tree is idle, pops the oldest and writes it to blackboard.last_command.
    If EMERGENCY_CANCEL appears, it's immediately promoted to last_command and buffer cleared.
    """

    def __init__(self, name="CommandManager"):
        super().__init__(name)
        self.buffer = []  # list of (stamp, command) tuples
        self._last_seen_stamp = None

    def setup(self, **kwargs):
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(
            key="last_command_received", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="command_tree_status", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="last_command", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="last_command_debug", access=py_trees.common.Access.WRITE
        )
        self.blackboard.last_command_debug = None
        # initialize if not present
        try:
            _ = self.blackboard.last_command_received
        except KeyError:
            self.blackboard.last_command_received = None
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
        cmd = self.blackboard.last_command_received
        if cmd is None:
            return Status.SUCCESS
        self.blackboard.last_command_debug = cmd.id
        # convert ROS Time to a comparable tuple
        stamp = getattr(cmd, 'stamp', None)
        if stamp is None:
            return Status.SUCCESS
        stamp_key = (stamp.sec, stamp.nanosec)

        # immediate emergency cancel
        last_cmd = self.blackboard.last_command
        if cmd.id == CommandID.EMERGENCY_CANCEL and (
            last_cmd is None or
            (last_cmd.stamp.sec, last_cmd.stamp.nanosec) != stamp_key
        ):
            self.buffer.clear()
            self.blackboard.last_command = cmd
            return Status.SUCCESS

        # insert new command into the buffer
        if stamp_key != self._last_seen_stamp:
            self._last_seen_stamp = stamp_key
            bisect.insort(self.buffer, (stamp_key, cmd))

        # if any buffered emergency cancel, fire it now
        for _, buffered_cmd in list(self.buffer):
            if buffered_cmd.id == CommandID.EMERGENCY_CANCEL:
                self.buffer.clear()
                self.blackboard.last_command = buffered_cmd
                return Status.SUCCESS

        # dispatch next when idle
        tree_status = self.blackboard.command_tree_status
        if self.buffer and tree_status != Status.RUNNING:
            _, next_cmd = self.buffer.pop(0)
            self.blackboard.last_command = next_cmd
            return Status.SUCCESS

        return Status.SUCCESS
