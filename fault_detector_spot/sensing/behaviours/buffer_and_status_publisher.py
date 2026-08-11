# In fault_detector_spot/behaviour_tree/nodes/command_status_publisher.py

import py_trees
from fault_detector_msgs.msg import CommandStatus
from py_trees.common import Status
from std_msgs.msg import String


class BufferStatusPublisher(py_trees.behaviour.Behaviour):
    """
    Every tick, reads:
      - blackboard.command_buffer  (a list of SimpleCommand)
      - blackboard.command_tree_status (a py_trees Status)
    and publishes them as JSON strings on ROS2 topics.
    """

    def __init__(self, name: str = "CmdStatusPub"):
        super().__init__(name)
        self.status_pub = None
        self.blackboard = None
        self.buffer_pub = None
        self.last_status = ""
        self.last_structured_status = None
        self.last_terminal_request_id = ""
        self.structured_status_pub = None
        self.node = None

    def setup(self, **kwargs) -> bool:
        self.node = kwargs['node']
        self.blackboard = self.attach_blackboard_client()
        self.buffer_pub = self.node.create_publisher(
            String,
            'fault_detector/command_buffer',
            10
        )
        self.status_pub = self.node.create_publisher(
            String,
            'fault_detector/command_tree_status',
            10
        )
        self.structured_status_pub = self.node.create_publisher(
            CommandStatus,
            'fault_detector/command_status',
            10,
        )
        # make sure keys exist
        self.blackboard.register_key("command_buffer", access=py_trees.common.Access.READ)
        self.blackboard.register_key("command_tree_status", access=py_trees.common.Access.READ)
        self.blackboard.register_key("last_command", access=py_trees.common.Access.READ)
        return True

    def update(self) -> Status:
        # read from the blackboard
        if self.blackboard.command_buffer is None:
            buffer_list = []
        else:
            buffer_list = self.blackboard.command_buffer

        # publish buffer as a list of IDs
        ids = [self._command_id_text(cmd) for cmd in buffer_list]
        buf_msg = String()
        buf_msg.data = f"[{','.join(ids)}]"
        self.buffer_pub.publish(buf_msg)

        structured_status = self.get_structured_status_message(
            len(buffer_list)
        )
        if self._follows_terminal_status(structured_status):
            return Status.SUCCESS

        stat_msg = self.get_status_message()
        if stat_msg.data != self.last_status:
            self.status_pub.publish(stat_msg)
            self.last_status = stat_msg.data

        signature = self._structured_status_signature(structured_status)
        if signature != self.last_structured_status:
            self.structured_status_pub.publish(structured_status)
            self.last_structured_status = signature
            if structured_status.state in (
                CommandStatus.STATE_SUCCEEDED,
                CommandStatus.STATE_FAILED,
            ):
                self.last_terminal_request_id = (
                    structured_status.request_id
                )

        return Status.SUCCESS

    def _follows_terminal_status(self, message):
        return bool(
            message.request_id
            and message.request_id == self.last_terminal_request_id
        )

    def get_status_message(self) -> str:
        stat = self.blackboard.command_tree_status
        name = String()
        if stat is None:
            name.data = "IDLE"
        elif stat == Status.RUNNING:
            name.data = (
                "Running: "
                f"{self._command_id_text(self.blackboard.last_command)}"
            )
        else:
            command = self.blackboard.last_command
            command_id = (
                self._command_id_text(command)
                if command is not None
                else "unknown"
            )
            name.data = f"{stat.name}: {command_id}"
        return name

    def get_structured_status_message(
        self,
        buffered_command_count=0,
    ) -> CommandStatus:
        """Return machine-readable state for the current request."""
        message = CommandStatus()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.buffered_command_count = buffered_command_count
        command = self.blackboard.last_command
        if command is not None:
            message.request_id = command.request_id
            message.command_id = self._command_id_text(command)

        status = self.blackboard.command_tree_status
        if status == Status.RUNNING:
            message.state = CommandStatus.STATE_RUNNING
        elif status == Status.SUCCESS:
            message.state = CommandStatus.STATE_SUCCEEDED
        elif status == Status.FAILURE:
            message.state = CommandStatus.STATE_FAILED
        else:
            message.state = CommandStatus.STATE_IDLE
        return message

    @staticmethod
    def _structured_status_signature(message):
        return (
            message.request_id,
            message.command_id,
            message.state,
            message.detail,
            message.buffered_command_count,
        )

    @staticmethod
    def _command_id_text(command) -> str:
        """Normalize plain strings and string-valued Enum identifiers."""
        command_id = command.command_id
        return str(getattr(command_id, "value", command_id))
