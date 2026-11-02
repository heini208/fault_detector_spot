import math
from copy import deepcopy

from fault_detector_spot.application.ros.semantic_command_adapter import semantic_command_to_message

import py_trees
from fault_detector_msgs.msg import CommandStatus
from py_trees.common import Status
from spot_msgs.msg import PowerState
from std_msgs.msg import Float32

from fault_detector_spot.shared.ros.qos_profiles import LATCHED_QOS
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.behaviour_tree.command_status_ledger import CommandStatusLedger


class BufferStatusPublisher(py_trees.behaviour.Behaviour):
    """Publish correlated command status and current robot battery percentage."""

    def __init__(self, name: str = "CmdStatusPub", *, status_ledger=None,
                 execution_idle=None, stop_runtime_idle=None):
        super().__init__(name)
        self.blackboard = None
        self.last_structured_status = None
        self.last_terminal_request_id = ""
        self.structured_status_pub = None
        self.battery_percentage_pub = None
        self.battery_state_sub = None
        self.latest_battery_percentage = None
        self.last_battery_percentage = None
        self.node = None
        self.status_ledger = status_ledger if status_ledger is not None else CommandStatusLedger()
        self.execution_idle = execution_idle
        self.stop_runtime_idle = stop_runtime_idle
        self._terminal_statuses = {}

    def setup(self, **kwargs) -> bool:
        self.node = kwargs["node"]
        self.blackboard = self.attach_blackboard_client()
        self.structured_status_pub = self.node.create_publisher(
            CommandStatus,
            "fault_detector/_internal/command_status",
            10,
        )
        self.battery_percentage_pub = self.node.create_publisher(
            Float32,
            "fault_detector/state/battery_percentage",
            LATCHED_QOS,
        )
        self.battery_state_sub = self.node.create_subscription(
            PowerState,
            "/status/power_states",
            self._process_power_state,
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
        self.blackboard.register_key(
            "command_failure_request_id",
            access=py_trees.common.Access.READ,
        )
        self.blackboard.register_key(
            "command_failure_detail",
            access=py_trees.common.Access.READ,
        )
        return True

    def update(self) -> Status:
        self._publish_battery_percentage_if_changed()

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
        if (structured_status.request_id
                and structured_status.command_id != CommandID.EMERGENCY_CANCEL.value):
            terminal = self._terminal_statuses.get(structured_status.request_id)
            if terminal is not None:
                structured_status = deepcopy(terminal)
                structured_status.execution_idle = (
                    buffered_request_count == 0 and self._idle_proven(self.execution_idle)
                )
            elif structured_status.state in (CommandStatus.STATE_SUCCEEDED, CommandStatus.STATE_FAILED):
                self._terminal_statuses[structured_status.request_id] = deepcopy(structured_status)
        structured_status = self.status_ledger.record(structured_status)
        signature = self._structured_status_signature(structured_status)
        if signature != self.last_structured_status:
            if structured_status.state == CommandStatus.STATE_STOP_CONFIRMED:
                messages = self.status_ledger.confirm_stop(structured_status)
            else:
                messages = [structured_status]
            for message in messages:
                self.structured_status_pub.publish(message)
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

    def _process_power_state(self, message: PowerState) -> None:
        percentage = float(message.locomotion_charge_percentage)
        if not math.isfinite(percentage):
            return
        self.latest_battery_percentage = min(100.0, max(0.0, percentage))

    def _publish_battery_percentage_if_changed(self) -> None:
        percentage = self.latest_battery_percentage
        if percentage is None or percentage == self.last_battery_percentage:
            return
        message = Float32()
        message.data = percentage
        self.battery_percentage_pub.publish(message)
        self.last_battery_percentage = percentage

    def _log_info(self, message: str) -> None:
        logger_factory = getattr(self.node, "get_logger", None)
        if not callable(logger_factory):
            return
        logger = logger_factory()
        info = getattr(logger, "info", None)
        if callable(info):
            info(message)

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

        detail = getattr(command, "execution_feedback_message", "")
        execution_detail = detail.strip() if isinstance(detail, str) else ""
        status = self.blackboard.command_tree_status
        if status == Status.RUNNING:
            message.state = CommandStatus.STATE_RUNNING
            message.detail = execution_detail or (
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
                message.detail = execution_detail or "BT request completed"
        elif status == Status.FAILURE:
            message.state = CommandStatus.STATE_FAILED
            message.retry_safe = bool(getattr(command, "retry_safe", False))
            message.continuation_safe = (
                message.retry_safe or bool(getattr(command, "continuation_safe", False))
            )
            retry_command = getattr(command, "retry_command", None)
            if retry_command is not None:
                message.has_retry_command = True
                message.retry_command = semantic_command_to_message(retry_command)
            failure_detail = self._current_failure_detail(message.request_id)
            if failure_detail:
                message.detail = (
                    f"{message.command_id} failed: {failure_detail}"
                    if message.command_id
                    else failure_detail
                )
            else:
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
        if message.command_id == CommandID.EMERGENCY_CANCEL.value:
            confirmed = (
                status == Status.SUCCESS
                and bool(getattr(command, "stop_confirmed", False))
                and self._idle_proven(self.execution_idle)
                and self._idle_proven(self.stop_runtime_idle)
                and buffered_command_count == 0
                and not self.blackboard.command_buffer
            )
            message.execution_idle = confirmed
            if confirmed:
                message.state = CommandStatus.STATE_STOP_CONFIRMED
                message.detail = "BT execution owners and runtimes confirmed stopped"
            elif status == Status.FAILURE or getattr(command, "stop_unconfirmed", False):
                message.state = CommandStatus.STATE_STOP_UNCONFIRMED
            else:
                message.state = CommandStatus.STATE_STOP_REQUESTED
            message.retry_safe = False
            message.continuation_safe = False
        elif message.request_id:
            if message.state in (CommandStatus.STATE_SUCCEEDED, CommandStatus.STATE_FAILED):
                message.execution_idle = (
                    buffered_command_count == 0 and self._idle_proven(self.execution_idle)
                )
        return message

    @staticmethod
    def _idle_proven(check):
        if check is None:
            return False
        try:
            return bool(check())
        except Exception:
            return False

    def _current_failure_detail(self, request_id: str) -> str:
        try:
            failure_request_id = self.blackboard.command_failure_request_id
            detail = self.blackboard.command_failure_detail
        except (AttributeError, KeyError):
            return ""
        if not request_id or failure_request_id != request_id:
            return ""
        return detail.strip() if isinstance(detail, str) else ""

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
            message.execution_idle,
            message.retry_safe,
            message.continuation_safe,
            message.has_retry_command,
            repr(message.retry_command) if message.has_retry_command else "",
        )

    @staticmethod
    def _command_id_text(command) -> str:
        """Normalize plain strings and string-valued Enum identifiers."""
        command_id = command.command_id
        return str(getattr(command_id, "value", command_id))
