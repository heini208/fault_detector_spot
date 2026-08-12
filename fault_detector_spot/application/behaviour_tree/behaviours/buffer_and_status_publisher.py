import py_trees
from fault_detector_msgs.msg import CommandStatus
from py_trees.common import Status


class BufferStatusPublisher(py_trees.behaviour.Behaviour):
    """Publish correlated internal status for the active semantic request."""

    def __init__(self, name: str = "CmdStatusPub"):
        super().__init__(name)
        self.blackboard = None
        self.last_structured_status = None
        self.last_terminal_request_id = ""
        self.structured_status_pub = None
        self.node = None

    def setup(self, **kwargs) -> bool:
        self.node = kwargs["node"]
        self.blackboard = self.attach_blackboard_client()
        self.structured_status_pub = self.node.create_publisher(
            CommandStatus,
            "fault_detector/_internal/command_status",
            10,
        )
        self.blackboard.register_key(
            "command_buffer",
            access=py_trees.common.Access.READ,
        )
        self.blackboard.register_key(
            "command_tree_status",
            access=py_trees.common.Access.READ,
        )
        self.blackboard.register_key(
            "last_command",
            access=py_trees.common.Access.READ,
        )
        return True

    def update(self) -> Status:
        if self.blackboard.command_buffer is None:
            buffer_list = []
        else:
            buffer_list = self.blackboard.command_buffer

        buffered_request_count = self._current_request_buffer_count(
            buffer_list
        )
        structured_status = self.get_structured_status_message(
            buffered_request_count
        )
        if self._follows_terminal_status(structured_status):
            return Status.SUCCESS

        signature = self._structured_status_signature(structured_status)
        if signature != self.last_structured_status:
            self.structured_status_pub.publish(structured_status)
            self.last_structured_status = signature
            if structured_status.request_id:
                self._log_info(
                    "BT status "
                    f"{structured_status.request_id}: "
                    f"{structured_status.detail}"
                )
            if structured_status.state in (
                CommandStatus.STATE_SUCCEEDED,
                CommandStatus.STATE_FAILED,
            ):
                self.last_terminal_request_id = (
                    structured_status.request_id
                )

        return Status.SUCCESS

    def _log_info(self, message: str) -> None:
        logger_factory = getattr(self.node, "get_logger", None)
        if not callable(logger_factory):
            return
        logger = logger_factory()
        info = getattr(logger, "info", None)
        if callable(info):
            info(message)

    def _follows_terminal_status(self, message):
        return bool(
            message.request_id
            and message.request_id == self.last_terminal_request_id
        )

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
            message.detail = (
                f"BT executing {message.command_id}"
                if message.command_id
                else "BT executing command"
            )
        elif status == Status.SUCCESS:
            if buffered_command_count > 0:
                message.state = CommandStatus.STATE_RUNNING
                message.detail = (
                    "BT command step succeeded; "
                    f"{buffered_command_count} buffered step(s) remain"
                )
            else:
                message.state = CommandStatus.STATE_SUCCEEDED
                message.detail = "BT request completed"
        elif status == Status.FAILURE:
            message.state = CommandStatus.STATE_FAILED
            message.detail = (
                f"BT execution failed for {message.command_id}"
                if message.command_id
                else "BT execution failed"
            )
        else:
            message.state = CommandStatus.STATE_IDLE
            if message.request_id:
                message.detail = "BT selected request; waiting for execution"
            else:
                message.detail = "BT idle"
        return message

    def _current_request_buffer_count(self, buffer_list):
        command = self.blackboard.last_command
        request_id = getattr(command, "request_id", "")
        if not request_id:
            return 0
        return sum(
            getattr(buffered, "request_id", "") == request_id
            for buffered in buffer_list
        )

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
