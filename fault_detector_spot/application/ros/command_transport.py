"""Connect the transport-independent command controller to ROS topics."""

from dataclasses import replace
import time

from rclpy.clock import Clock, ClockType

from fault_detector_spot.application.commanding.command_lifecycle import CommandLifecycleConfig

from fault_detector_msgs.msg import (
    CommandRequest as CommandRequestMessage,
    CommandStatus,
    SoftwareStopState,
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
from fault_detector_spot.shared.ros.qos_profiles import COMMAND_REQUEST_QOS, LATCHED_QOS
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
        query_timeout_sec=2.0,
        monotonic_clock=time.monotonic,
    ):
        if not isinstance(controller, CommandController):
            raise TypeError("Expected a CommandController")
        limits = CommandLifecycleConfig(poll_period_sec=poll_period_sec,
                                        query_timeout_sec=query_timeout_sec)
        if not callable(monotonic_clock):
            raise TypeError("Monotonic clock must be callable")
        self._monotonic_clock = monotonic_clock
        self._query_timeout_sec = limits.query_timeout_sec
        self._heartbeat_timeout_sec = controller.lifecycle_limits.heartbeat_timeout_sec
        self._reconciliation_interval_sec = controller.lifecycle_limits.reconciliation_interval_sec
        self._remote_instance_id = ""
        self._retired_instances = set()
        self._remote_last_receipt = None
        self._remote_status_sequence = 0
        self._remote_query_sequence = 0
        self._remote_accepting_commands = False
        self._remote_stop_latched = False
        self._remote_stop_confirmed = False
        self._last_discovery = None
        self._request_instances = {}
        self.node = node
        self.controller = controller
        self._local_dispatch = None
        self._local_query_status = None
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
        self._software_stop_publisher = node.create_publisher(
            SoftwareStopState, "fault_detector/application/software_stop_state", LATCHED_QOS,
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
            limits.poll_period_sec,
            self._poll,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self.controller.add_accepted_listener(self._publish_accepted)
        self.controller.add_status_listener(self._publish_controller_status)
        self.controller.add_software_stop_listener(self._publish_software_stop)
        self.controller.configure_dispatch(
            self._dispatch,
            self._dispatch_consumer_ready,
            emergency_ready=self._emergency_consumer_ready,
            reset_ready=self._reset_consumer_ready,
        )
        self.controller.configure_reconciliation(self._query_status)
        self._discover_owner()

    def set_local_dispatch(self, dispatch, *, query_status=None) -> None:
        """Set the optional in-process command handler."""
        if dispatch is not None and not callable(dispatch):
            raise TypeError("Local dispatch must be callable")
        if query_status is not None and not callable(query_status):
            raise TypeError("Local status query must be callable")
        self._local_dispatch = dispatch
        self._local_query_status = query_status

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
        received_at = self._monotonic_clock()
        if (self._closed or message.request_id in self._local_request_ids
                or not self._remote_instance_id
                or message.execution_instance_id != self._remote_instance_id
                or message.status_sequence <= self._remote_status_sequence):
            return False
        self._remote_status_sequence = message.status_sequence
        return self._apply_execution_status(message, received_at)

    def _apply_execution_status(self, message, received_at):
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
                execution_instance_id=message.execution_instance_id,
                status_sequence=message.status_sequence,
                software_stop_latched=message.software_stop_latched,
                software_stop_confirmed=message.software_stop_confirmed,
            ),
            receipt_monotonic=received_at,
        )

    def close(self) -> None:
        """Detach controller callbacks owned by this ROS transport."""
        self._closed = True
        self.controller.clear_dispatch()
        self.controller.remove_accepted_listener(self._publish_accepted)
        self.controller.remove_status_listener(self._publish_controller_status)
        self.controller.remove_software_stop_listener(self._publish_software_stop)
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
            targets = [(key, owner) for key, owner in zip(
                request.stop_request_ids, request.stop_execution_instance_ids,
            ) if key not in self._local_request_ids]
            self._publish_dispatch(replace(
                request, stop_request_ids=tuple(key for key, _ in targets),
                stop_execution_instance_ids=tuple(owner for _, owner in targets),
            ))
            return
        if request.command.command_id is not CommandID.RESET_SOFTWARE_STOP and self._local_dispatch is not None:
            # Scheduling may raise after work starts; retain that local ownership.
            self._local_request_ids.add(request.request_id)
            if self._local_dispatch(request):
                return
            self._local_request_ids.discard(request.request_id)
        self._publish_dispatch(request)

    def _poll(self):
        if self._closed:
            return
        self._discover_owner()
        self.controller.poll()

    def _discover_owner(self):
        now = self._monotonic_clock()
        if self._last_discovery is None or now - self._last_discovery >= self._reconciliation_interval_sec:
            self._last_discovery = now
            try:
                self._query_status("")
            except Exception as exception:
                # Discovery failures must not prevent command deadlines ticking.
                self.node.get_logger().error(f"Execution owner discovery failed: {exception}")

    def _query_status(self, request_id):
        """Fresh query replies are heartbeats; queries never resend commands."""
        if self._closed:
            return
        if request_id in self._local_request_ids:
            if self._local_query_status is not None:
                self._local_query_status(request_id)
            return
        pending = self._status_queries.get(request_id)
        now = self._monotonic_clock()
        if pending is not None:
            if now - pending[1] < self._query_timeout_sec:
                return
            self._status_client.remove_pending_request(pending[0])
            self._status_queries.pop(request_id, None)
        if not self._status_client.service_is_ready():
            return
        expected_instance = self._request_instances.get(request_id, "")
        future = self._status_client.call_async(GetCommandStatus.Request(
            request_id=request_id, execution_instance_id=expected_instance,
        ))
        self._status_queries[request_id] = (future, now)

        def completed(result_future):
            received_at = self._monotonic_clock()
            current = self._status_queries.get(request_id)
            if self._closed or current is None or current[0] is not result_future:
                return
            self._status_queries.pop(request_id, None)
            # A late queued reply is not evidence that the owner is alive now.
            if received_at - current[1] >= self._query_timeout_sec:
                return
            try:
                response = result_future.result()
                status = response.status
                if status.request_id != request_id or not self._observe_owner(response, received_at):
                    return
                if request_id:
                    self._apply_execution_status(status, received_at)
            except Exception as exception:
                self.node.get_logger().error(f"Command status query failed: {exception}")

        future.add_done_callback(completed)

    def _observe_owner(self, response, received_at):
        """Only a fresh service response may establish or replace an owner."""
        status = response.status
        instance = status.execution_instance_id
        if not instance or instance in self._retired_instances or status.status_sequence <= 0:
            return False
        previous = self._remote_instance_id
        if previous != instance:
            # Replies begun before this observation cannot establish boot order.
            # Invalidate them, including replies from an intermediate restart we
            # have not seen, and challenge the current owner afresh next time.
            pending = tuple(self._status_queries.values())
            self._status_queries.clear()
            for future, _ in pending:
                self._status_client.remove_pending_request(future)
            if previous:
                self._retired_instances.add(previous)
            self._remote_instance_id = instance
            self._remote_status_sequence = 0
            self._remote_query_sequence = 0
            self._remote_accepting_commands = False
            self._remote_stop_latched = False
            self._remote_stop_confirmed = False
            self.controller.execution_owner_changed(previous, instance)
        if status.status_sequence > self._remote_query_sequence:
            self._remote_query_sequence = status.status_sequence
            self._remote_last_receipt = received_at
        if status.status_sequence > self._remote_status_sequence:
            self._remote_status_sequence = status.status_sequence
            self._remote_accepting_commands = bool(
                response.accepting_commands and not response.unresolved_request_ids
            )
            self._remote_stop_latched = status.software_stop_latched
            self._remote_stop_confirmed = status.software_stop_confirmed
            self.controller.observe_owner_software_stop(self._remote_stop_latched, self._remote_stop_confirmed)
        # A deadline-valid query can arrive after a newer topic snapshot. It is
        # still live owner evidence, but must not replace newer global occupancy;
        # the controller independently rejects older state for the same request.
        return True

    def _publish_dispatch(self, request) -> None:
        instance = self._remote_instance_id
        if not instance:
            raise RuntimeError("Execution owner has not been discovered")
        self.controller.bind_execution_owner(request.request_id, instance)
        self._request_instances[request.request_id] = instance
        message = command_request_to_message(replace(request, execution_instance_id=instance))
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

    def _publish_software_stop(self, status):
        message = SoftwareStopState()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.request_id = status.request_id
        message.latched = status.latched
        message.stop_confirmed = status.stop_confirmed
        message.detail = status.detail
        message.status_sequence = status.status_sequence
        self._software_stop_publisher.publish(message)

    def _publish_controller_status(
        self,
        status: CommandControllerStatus,
    ) -> None:
        message = CommandStatus()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.request_id = status.request_id
        message.execution_instance_id = status.execution_instance_id
        message.status_sequence = status.status_sequence
        message.command_id = status.request.command.command_id.value
        message.state = self._controller_status_state(status.state)
        message.detail = status.detail or status.state.value
        message.buffered_command_count = status.buffered_command_count
        message.retry_safe = status.retry_safe
        message.continuation_safe = status.continuation_safe
        message.execution_idle = status.execution_idle
        stop = self.controller.software_stop_status
        message.software_stop_latched = stop.latched
        message.software_stop_confirmed = stop.stop_confirmed
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
        return bool(
            self._remote_accepting_commands and self._remote_last_receipt is not None
            and self._monotonic_clock() - self._remote_last_receipt < self._heartbeat_timeout_sec
            and self._emergency_consumer_ready()
        )

    def _reset_consumer_ready(self) -> bool:
        # Reset is the one nonmotion command admitted by a latched owner.
        # The owner revalidates physical stopping when executing the reset.
        return bool(
            self._remote_stop_latched and self._remote_stop_confirmed
            and self._remote_last_receipt is not None
            and self._monotonic_clock() - self._remote_last_receipt < self._heartbeat_timeout_sec
            and self._emergency_consumer_ready()
        )

    def _emergency_consumer_ready(self) -> bool:
        # Loss of the query channel must not prevent an explicit stop attempt.
        # Dispatch still targets the discovered instance; a replacement rejects
        # that envelope and neither instance can release the old lane by silence.
        if not self._remote_instance_id:
            return False
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
