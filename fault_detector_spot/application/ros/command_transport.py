"""Connect the transport-independent command controller to ROS topics."""

from fault_detector_msgs.msg import (
    CommandRequest as CommandRequestMessage,
    CommandStatus,
)

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
            self._publish_dispatch,
            self._dispatch_consumer_ready,
        )

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
        values = {
            CommandStatus.STATE_RUNNING: CommandControllerState.RUNNING,
            CommandStatus.STATE_SUCCEEDED: CommandControllerState.SUCCEEDED,
            CommandStatus.STATE_FAILED: CommandControllerState.FAILED,
            CommandStatus.STATE_CANCELLED: CommandControllerState.CANCELLED,
        }
        state = values.get(message.state)
        if state is None:
            return False
        return self.controller.handle_execution_status(
            CommandExecutionStatus(
                request_id=message.request_id,
                state=state,
                detail=message.detail,
                buffered_command_count=message.buffered_command_count,
            )
        )

    def close(self) -> None:
        """Detach controller callbacks owned by this ROS transport."""
        self.controller.clear_dispatch()
        self.controller.remove_accepted_listener(self._publish_accepted)
        self.controller.remove_status_listener(self._publish_controller_status)
        cancel = getattr(self._poll_timer, "cancel", None)
        if callable(cancel):
            cancel()

    def _publish_dispatch(self, request) -> None:
        message = command_request_to_message(request)
        message.command.command.header.stamp = self.node.get_clock().now().to_msg()
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
        self._controller_status_publisher.publish(message)

    def _reject_message(self, message, detail) -> None:
        command = getattr(message, "command", None)
        basic = getattr(command, "command", None)
        request_id = getattr(message, "request_id", "")
        command_id = getattr(basic, "command_id", "")
        self.node.get_logger().error(
            f"Rejected command request: {detail}"
        )
        if not request_id:
            return
        status = CommandStatus()
        status.header.stamp = self.node.get_clock().now().to_msg()
        status.request_id = request_id
        status.command_id = command_id
        status.state = CommandStatus.STATE_FAILED
        status.detail = detail
        status.buffered_command_count = 0
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
        raise ValueError(f"Unsupported controller state: {state}")


__all__ = ["RosCommandTransport"]
