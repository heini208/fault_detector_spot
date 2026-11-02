"""Connect the transport-independent command controller to ROS topics."""

from dataclasses import replace
import time

from fault_detector_msgs.msg import (
    CommandRequest as CommandRequestMessage,
    CommandStatus,
)
from fault_detector_msgs.srv import GetCommandStatus

from fault_detector_spot.application.controllers.command_controller import (
    CommandController,
    CommandControllerState,
    CommandControllerStatus,
    CommandExecutionStatus,
)
from fault_detector_spot.application.ros.command_request_adapter import (
    command_request_from_message,
    command_request_to_message,
)
from fault_detector_spot.shared.ros.qos_profiles import COMMAND_REQUEST_QOS
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.ros.semantic_command_adapter import (
    semantic_command_from_message, semantic_command_to_message,
)


class RosCommandTransport:
    """Own ROS transport for one application command controller."""

    def __init__(
        self,
        node,
        controller: CommandController,
        submission_topic="fault_detector/_internal/commands/submit",
        dispatch_topic="fault_detector/_internal/commands/request",
        accepted_topic="fault_detector/_internal/commands/accepted",
        controller_status_topic="fault_detector/_internal/commands/status",
        status_topic="fault_detector/_internal/command_status",
        poll_period_sec=0.1,
    ):
        if not isinstance(controller, CommandController):
            raise TypeError("Expected a CommandController")
        self.node = node
        self.controller = controller
        self._local_dispatch = None
        self._local_request_ids = set()
        self._status_queries = {}
        self._closed = False
        self._status_client = node.create_client(
            GetCommandStatus, "fault_detector/_internal/commands/get_status",
        )
        self._dispatch_publisher = node.create_publisher(
            CommandRequestMessage,
            dispatch_topic,
            COMMAND_REQUEST_QOS,
        )
        self._accepted_publisher = node.create_publisher(
            CommandRequestMessage,
            accepted_topic,
            COMMAND_REQUEST_QOS,
        )
        self._controller_status_publisher = node.create_publisher(
            CommandStatus,
            controller_status_topic,
            10,
        )
        self._submission_subscription = node.create_subscription(
            CommandRequestMessage,
            submission_topic,
            self.submit_message,
            COMMAND_REQUEST_QOS,
        )
        self._status_subscription = node.create_subscription(
            CommandStatus,
            status_topic,
            self.handle_command_status,
            10,
        )
        self._poll_timer = node.create_timer(
            float(poll_period_sec),
            self.controller.poll,
        )
        self.controller.add_accepted_listener(self._publish_accepted)
        self.controller.add_status_listener(self._publish_controller_status)
        self.controller.configure_dispatch(
            self._dispatch,
            self._dispatch_consumer_ready,
        )
        self.controller.configure_reconciliation(self._query_status)

    def set_local_dispatch(self, dispatch) -> None:
        """Set the optional in-process command handler."""
        if dispatch is not None and not callable(dispatch):
            raise TypeError("Local dispatch must be callable")
        self._local_dispatch = dispatch

    def submit_message(self, message: CommandRequestMessage) -> bool:
        """Validate one ROS submission before it enters the command queue."""
        try:
            request = command_request_from_message(message)
            self.controller.submit(request)
        except (TypeError, ValueError) as exception:
            self._reject_message(message, str(exception))
            return False
        return True

    def handle_command_status(self, message: CommandStatus) -> bool:
        """Translate behavior-tree feedback into controller feedback."""
        if message.request_id in self._local_request_ids:
            return False
        values = {
            CommandStatus.STATE_RUNNING: CommandControllerState.RUNNING,
            CommandStatus.STATE_SUCCEEDED: CommandControllerState.SUCCEEDED,
            CommandStatus.STATE_FAILED: CommandControllerState.FAILED,
            CommandStatus.STATE_CANCELLED: CommandControllerState.CANCELLED,
            CommandStatus.STATE_UNKNOWN: CommandControllerState.UNKNOWN,
            CommandStatus.STATE_STOP_REQUESTED: CommandControllerState.STOP_REQUESTED,
            CommandStatus.STATE_STOP_CONFIRMED: CommandControllerState.STOP_CONFIRMED,
            CommandStatus.STATE_STOP_UNCONFIRMED: CommandControllerState.STOP_UNCONFIRMED,
        }
        state = values.get(message.state)
        if state is None:
            return False
        retry_command = None
        retry_safe = message.retry_safe
        detail = message.detail
        if state is CommandControllerState.FAILED and message.has_retry_command:
            try:
                retry_command = semantic_command_from_message(message.retry_command)
            except (TypeError, ValueError) as exception:
                retry_safe = False
                detail += f"; invalid retry command: {exception}"
        return self.controller.handle_execution_status(
            CommandExecutionStatus(
                request_id=message.request_id,
                state=state,
                detail=detail,
                buffered_command_count=message.buffered_command_count,
                retry_safe=retry_safe,
                retry_command=retry_command,
                continuation_safe=message.continuation_safe,
                execution_idle=message.execution_idle,
            )
        )

    def close(self) -> None:
        """Detach controller callbacks owned by this ROS transport."""
        self._closed = True
        self.controller.clear_dispatch()
        self.controller.remove_accepted_listener(self._publish_accepted)
        self.controller.remove_status_listener(self._publish_controller_status)
        cancel = getattr(self._poll_timer, "cancel", None)
        if callable(cancel):
            cancel()
        pending = tuple(self._status_queries.values())
        self._status_queries.clear()
        for future, _ in pending:
            self._status_client.remove_pending_request(future)
        self.node.destroy_client(self._status_client)

    def _dispatch(self, request) -> None:
        if request.command.command_id is CommandID.EMERGENCY_CANCEL:
            self._publish_dispatch(replace(request, stop_request_ids=tuple(
                request_id for request_id in request.stop_request_ids
                if request_id not in self._local_request_ids
            )))
            return
        if self._local_dispatch is not None:
            # Keep ownership local even when a local submission raises after
            # scheduling work. A BT reply cannot certify that work stopped.
            self._local_request_ids.add(request.request_id)
            if self._local_dispatch(request):
                return
            self._local_request_ids.discard(request.request_id)
        self._publish_dispatch(request)

    def _query_status(self, request_id):
        """Recover lost owner feedback without republishing a motion request."""
        if self._closed or request_id in self._local_request_ids:
            return
        pending = self._status_queries.get(request_id)
        now = time.monotonic()
        if pending is not None:
            if now - pending[1] < 3.0:
                return
            self._status_client.remove_pending_request(pending[0])
            self._status_queries.pop(request_id, None)
        if not self._status_client.service_is_ready():
            return
        future = self._status_client.call_async(GetCommandStatus.Request(request_id=request_id))
        self._status_queries[request_id] = (future, now)

        def completed(result_future):
            current = self._status_queries.get(request_id)
            if current is None or current[0] is not result_future:
                return
            self._status_queries.pop(request_id, None)
            try:
                response = result_future.result()
                if response.status.request_id == request_id:
                    self.handle_command_status(response.status)
            except Exception as exception:
                self.node.get_logger().error(f"Command status query failed: {exception}")

        future.add_done_callback(completed)

    def _publish_dispatch(self, request) -> None:
        message = command_request_to_message(request)
        message.header.stamp = self.node.get_clock().now().to_msg()
        self._dispatch_publisher.publish(message)
        self._log_info(
            "Published "
            f"{request.command.command_id.value} to behavior tree "
            f"[{request.request_id}]"
        )

    def _publish_accepted(self, request) -> None:
        self._accepted_publisher.publish(
            command_request_to_message(request)
        )

    def _publish_controller_status(
        self,
        status: CommandControllerStatus,
    ) -> None:
        message = CommandStatus()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.request_id = status.request_id
        message.command_id = status.request.command.command_id.value
        message.state = self._controller_status_state(status.state)
        message.detail = status.detail or status.state.value
        message.buffered_command_count = status.buffered_command_count
        message.retry_safe = status.retry_safe
        message.continuation_safe = status.continuation_safe
        message.execution_idle = status.execution_idle
        if status.retry_command is not None:
            message.has_retry_command = True
            message.retry_command = semantic_command_to_message(status.retry_command)
        self._controller_status_publisher.publish(message)

    def _reject_message(self, message, detail) -> None:
        request_id = getattr(message, "request_id", "")
        payload = getattr(message, "payload", None)
        command_id = getattr(payload, "command_id", "")
        self.node.get_logger().error(
            f"Rejected command request: {detail}"
        )
        if not request_id:
            return
        if self.controller.knows_request(request_id):
            return
        status = CommandStatus()
        status.header.stamp = self.node.get_clock().now().to_msg()
        status.request_id = request_id
        status.command_id = command_id
        status.state = CommandStatus.STATE_FAILED
        status.detail = detail
        status.buffered_command_count = 0
        status.execution_idle = True
        self._controller_status_publisher.publish(status)

    def _dispatch_consumer_ready(self) -> bool:
        get_count = getattr(
            self._dispatch_publisher,
            "get_subscription_count",
            None,
        )
        if not callable(get_count):
            return True
        try:
            return int(get_count()) > 0
        except Exception:
            return False

    def _log_info(self, message: str) -> None:
        get_logger = getattr(self.node, "get_logger", None)
        if not callable(get_logger):
            return
        info = getattr(get_logger(), "info", None)
        if callable(info):
            info(message)

    @staticmethod
    def _controller_status_state(state):
        if state in (
            CommandControllerState.QUEUED,
            CommandControllerState.DISPATCHED,
            CommandControllerState.RUNNING,
        ):
            return CommandStatus.STATE_RUNNING
        if state is CommandControllerState.SUCCEEDED:
            return CommandStatus.STATE_SUCCEEDED
        if state is CommandControllerState.FAILED:
            return CommandStatus.STATE_FAILED
        if state is CommandControllerState.CANCELLED:
            return CommandStatus.STATE_CANCELLED
        if state is CommandControllerState.UNKNOWN:
            return CommandStatus.STATE_UNKNOWN
        if state is CommandControllerState.STOP_REQUESTED:
            return CommandStatus.STATE_STOP_REQUESTED
        if state is CommandControllerState.STOP_CONFIRMED:
            return CommandStatus.STATE_STOP_CONFIRMED
        if state is CommandControllerState.STOP_UNCONFIRMED:
            return CommandStatus.STATE_STOP_UNCONFIRMED
        raise ValueError(f"Unsupported controller state: {state}")


__all__ = ["RosCommandTransport"]
